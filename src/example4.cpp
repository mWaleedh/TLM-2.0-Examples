#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/peq_with_cb_and_phase.h>
#include <vector>

using namespace std;
using namespace tlm;
using namespace tlm_utils;
using namespace sc_dt;

// Memory manager for dynamic memory. Used because:
// In Non-Blocking transactions, the calling function could end before the transaction completes.
class MemoryManager : public tlm_mm_interface {
    vector<tlm_generic_payload*> free_list;
public:
    tlm_generic_payload* allocate() {
        if (free_list.empty()) {
            return new tlm_generic_payload(this);
        }
        tlm_generic_payload* trans = free_list.back();
        free_list.pop_back();
        return trans;
    }

    void free(tlm_generic_payload* trans) {
        trans->reset();
        free_list.push_back(trans);
    }
};

struct Initiator : sc_module {
    simple_initiator_socket<Initiator> socket;
    peq_with_cb_and_phase<Initiator> m_peq;
    MemoryManager m_mm;
    tlm_generic_payload* request_in_progress;
    sc_event end_request_event;

    void initiator_thread() {
        tlm_generic_payload* trans;
        sc_time delay;

        for (int i = 0; i < 2; i++) {
            // Allocate from memory manager
            trans = m_mm.allocate();
            trans->acquire();   // Increment reference count

            int* data = new int(0xAABBCCDD + i);
            trans->set_command(TLM_WRITE_COMMAND);
            trans->set_address(0x10 + (i * 4));
            trans->set_data_ptr(reinterpret_cast<unsigned char*>(data));
            trans->set_data_length(4);
            trans->set_streaming_width(4);
            trans->set_byte_enable_ptr(nullptr);
            trans->set_dmi_allowed(false);
            trans->set_response_status(TLM_INCOMPLETE_RESPONSE);

            if (request_in_progress) {
                wait(end_request_event);
            }
            request_in_progress = trans;

            // Send BEGIN_REQ
            tlm_phase phase = BEGIN_REQ;
            delay = SC_ZERO_TIME;
            
            cout << "@" << sc_time_stamp() << " Sending BEGIN_REQ for address 0x" << hex << trans->get_address() << endl << endl;
            
            tlm_sync_enum status = socket->nb_transport_fw(*trans, phase, delay);

            // Check status
            if (status == TLM_UPDATED) {
                // Annotate delay
                m_peq.notify(*trans, phase, delay);
            } 
            else if (status == TLM_COMPLETED) {
                request_in_progress = 0;
            }
        }
    }

    // Receives END_REQ and BEGIN_RESP
    virtual tlm_sync_enum nb_transport_bw(tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay) {
        m_peq.notify(trans, phase, delay);
        return TLM_ACCEPTED; 
    }

    // Wakes up when delay expires
    void peq_cb(tlm_generic_payload& trans, const tlm_phase& phase) {
        if (phase == END_REQ || (&trans == request_in_progress && phase == tlm::BEGIN_RESP)) {
            request_in_progress = 0;
            end_request_event.notify();     // Notify bus is empty
        }

        if (phase == END_REQ) {
            cout << "@" << sc_time_stamp() << " Received END_REQ. Bus is free." << endl;
        } 
        else if (phase == BEGIN_RESP) {
            cout << "@" << sc_time_stamp() << " Received BEGIN_RESP. Transaction successful." << endl;
            
            // Send END_RESP to Target
            tlm_phase fw_phase = END_RESP;
            sc_time delay = SC_ZERO_TIME;
            
            cout << "@" << sc_time_stamp() << " Sending END_RESP." << endl << endl;
            socket->nb_transport_fw(trans, fw_phase, delay);

            trans.release();    // Decrement reference count
        }
    }

    SC_CTOR(Initiator) : socket("socket"), m_peq(this, &Initiator::peq_cb), request_in_progress(0) {
        socket.register_nb_transport_bw(this, &Initiator::nb_transport_bw);
        SC_THREAD(initiator_thread);
    }
};

// Internal phase
DECLARE_EXTENDED_PHASE(internal_ph);

struct Target : sc_module {
    simple_target_socket<Target> socket;
    peq_with_cb_and_phase<Target> m_peq;

    // 64 * 4 = 256 bytes
    static constexpr int SIZE = 64;
    int mem[SIZE];

    int n_trans;
    bool response_in_progress;
    tlm_generic_payload* next_response_pending;
    tlm_generic_payload* end_req_pending;

    // Only check for errors and then call PEQ
    virtual tlm_sync_enum nb_transport_fw(tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay) {
        unsigned int len = trans.get_data_length();
        unsigned char* byt = trans.get_byte_enable_ptr();
        unsigned int wid = trans.get_streaming_width();

        // Check for unsupported features
        if (byt != 0) {
            trans.set_response_status(TLM_BYTE_ENABLE_ERROR_RESPONSE);
            return TLM_COMPLETED;
        }
        if (len > 4 || wid < len) {
            trans.set_response_status(TLM_BURST_ERROR_RESPONSE);
            return TLM_COMPLETED;
        }

        m_peq.notify(trans, phase, delay);
        return TLM_ACCEPTED;    // Notify initiator that its request has been received
    }

    void peq_cb(tlm_generic_payload& trans, const tlm_phase& phase) {
        tlm_sync_enum status;

        switch (phase) {
            case BEGIN_REQ:
                trans.acquire();

                // Pipeline only handles 2 transactions at a time
                // If there is no space queue transaction
                if (n_trans == 2) {
                    end_req_pending = &trans;
                } 
                else {
                    status = send_end_req(trans);
                    if (status == TLM_COMPLETED) {
                        break;
                    }
                }
                break;

            case END_RESP:
                // Premature END_RESP
                if (!response_in_progress) {
                    SC_REPORT_FATAL("TLM-2", "Illegal END_RESP received");
                }

                trans.release();
                n_trans--;

                // Bus is clear so push next response if waiting
                response_in_progress = false;
                if (next_response_pending) {
                    send_response(*next_response_pending);
                    next_response_pending = 0;
                }

                // Unblock initiator if a request was waiting for room
                if (end_req_pending) {
                    status = send_end_req(*end_req_pending);
                    end_req_pending = 0;
                }
                break;

            // Not set by Initiator
            case END_REQ:
            case BEGIN_RESP:
                SC_REPORT_FATAL("TLM-2", "Illegal phase received by target");
                break;

            default:
                if (phase == internal_ph) {
                    // Execute Read/Write
                    tlm_command cmd = trans.get_command();
                    uint64 addr = trans.get_address() / 4;
                    unsigned char* ptr = trans.get_data_ptr();
                    unsigned int len = trans.get_data_length();

                    if (addr < SIZE) {
                        if (cmd == TLM_READ_COMMAND) {
                            memcpy(ptr, &mem[addr], len);
                        } 
                        else if (cmd == TLM_WRITE_COMMAND) {
                            memcpy(&mem[addr], ptr, len);
                        }
                        trans.set_response_status(TLM_OK_RESPONSE);
                    } 
                    else {
                        trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
                    }

                    // Target can't send BEGIN_RESP until previous transaction has been completed
                    if (response_in_progress) {
                        // Only two transactions are allowed
                        if (next_response_pending) {
                            SC_REPORT_FATAL("TLM-2", "Two pending responses in target");
                        }
                        next_response_pending = &trans;
                    } 
                    else {
                        send_response(trans);
                    }
                    break;
                }
        }
    }

    // Sends back END_REQ and moves transaction to internal execution phase
    tlm_sync_enum send_end_req(tlm_generic_payload& trans) {
        tlm_phase bw_phase = END_REQ;
        sc_time delay = sc_time(10, SC_NS); // Simulated delay
        
        tlm_sync_enum status = socket->nb_transport_bw(trans, bw_phase, delay);
        if (status == TLM_COMPLETED) {
            trans.release();
            return status;
        }

        // Queue internal memory execution
        m_peq.notify(trans, internal_ph, delay + sc_time(20, SC_NS));   // Add execution latency
        n_trans++;
        return status;
    }

    // Sends BEGIN_RESP back to Initiator
    void send_response(tlm_generic_payload& trans) {
        tlm_phase bw_phase = BEGIN_RESP;
        sc_time delay = SC_ZERO_TIME;
        
        response_in_progress = true;
        socket->nb_transport_bw(trans, bw_phase, delay);
    }

    SC_CTOR(Target) : socket("socket"), m_peq(this, &Target::peq_cb) {
        socket.register_nb_transport_fw(this, &Target::nb_transport_fw);
        for (int i = 0; i < SIZE; i++) mem[i] = 0;
        
        n_trans = 0;
        response_in_progress = false;
        next_response_pending = 0;
        end_req_pending = 0;
    }
};

int sc_main(int argc, char* argv[]) {
    Initiator initiator("initiator");
    Target target("target");

    initiator.socket.bind(target.socket);

    sc_start();
    return 0;
}