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

// Payload extension adding initiator number
struct src_id_extension : tlm_extension<src_id_extension> {
    int initiator_id;

    virtual tlm_extension_base* clone() const {
        src_id_extension* t = new src_id_extension;
        t->initiator_id = this->initiator_id;
        return t;
    }

    virtual void copy_from(tlm_extension_base const &ext) {
        initiator_id = static_cast<src_id_extension const &>(ext).initiator_id;
    }
};

struct Initiator : sc_module {
    simple_initiator_socket<Initiator> socket;
    peq_with_cb_and_phase<Initiator> m_peq;
    MemoryManager m_mm;
    
    tlm_generic_payload* request_in_progress;
    sc_event end_request_event;
    int id;
    uint64 base_addr;

    void initiator_thread() {
        tlm_generic_payload* trans;
        sc_time delay;

        // Initiate 2 transactions
        for (int i = 0; i < 2; i++) {
            trans = m_mm.allocate();
            trans->acquire();   

            int* data = new int(0xAABBCCDD + i + (id * 0x1000));
            trans->set_command(TLM_WRITE_COMMAND);
            trans->set_address(base_addr + (i * 4));
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

            tlm_phase phase = BEGIN_REQ;
            delay = SC_ZERO_TIME;
            
            cout << "@" << sc_time_stamp() << " Init " << id << ": Sending BEGIN_REQ for address 0x" << hex << trans->get_address() << dec << endl << endl;
            
            tlm_sync_enum status = socket->nb_transport_fw(*trans, phase, delay);

            if (status == TLM_UPDATED) {
                m_peq.notify(*trans, phase, delay);
            } 
            else if (status == TLM_COMPLETED) {
                request_in_progress = 0;
            }
        }
    }

    virtual tlm_sync_enum nb_transport_bw(tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay) {
        m_peq.notify(trans, phase, delay);
        return TLM_ACCEPTED; 
    }

    void peq_cb(tlm_generic_payload& trans, const tlm_phase& phase) {
        if (phase == END_REQ || (&trans == request_in_progress && phase == tlm::BEGIN_RESP)) {
            request_in_progress = 0;
            end_request_event.notify();
        }

        if (phase == END_REQ) {
            cout << "@" << sc_time_stamp() << " Init " << id << ": Received END_REQ for 0x" << hex << trans.get_address() << dec << ". Bus is free" << endl << endl;
        } 
        else if (phase == BEGIN_RESP) {
            cout << "@" << sc_time_stamp() << " Init " << id << ": Received BEGIN_RESP for 0x" << hex << trans.get_address() << dec << ". Transaction successful" << endl << endl;
            
            tlm_phase fw_phase = END_RESP;
            sc_time delay = SC_ZERO_TIME;
            
            socket->nb_transport_fw(trans, fw_phase, delay);
            trans.release();
        }
    }

    SC_HAS_PROCESS(Initiator);
    Initiator(sc_module_name name, int _id, uint64 _base_addr) : sc_module(name), socket("socket"), m_peq(this, &Initiator::peq_cb), request_in_progress(0), id(_id), base_addr(_base_addr) {
        socket.register_nb_transport_bw(this, &Initiator::nb_transport_bw);
        SC_THREAD(initiator_thread);
    }
};

struct Interconnect : sc_module {
    simple_target_socket_tagged<Interconnect> target_socket[2];
    simple_initiator_socket_tagged<Interconnect> initiator_socket[2];

    virtual tlm_sync_enum nb_transport_fw(int id, tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay) {
        // If its a new transaction assign the correct initiator id
        if (phase == BEGIN_REQ) {
            src_id_extension* ext = new src_id_extension;
            ext->initiator_id = id;
            
            // Will be automatically deleted
            trans.set_auto_extension(ext); 
        }

        uint64 global_addr = trans.get_address();
        int target_id;
        uint64 target_base;

        // Decode address
        if (global_addr <= 0xFF) { 
            target_id = 0; 
            target_base = 0x0; 
        } 
        else if (global_addr >= 0x100 && global_addr <= 0x1FF) { 
            target_id = 1; 
            target_base = 0x100; 
        } 
        else {
            trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
            return TLM_COMPLETED;
        }

        // Translate address and forward
        trans.set_address(global_addr - target_base);
        tlm_sync_enum status = initiator_socket[target_id]->nb_transport_fw(trans, phase, delay);
        
        // Restore global address
        trans.set_address(global_addr);
        return status;
    }

    virtual tlm_sync_enum nb_transport_bw(int id, tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay) {
        // Retrieve extension
        src_id_extension* ext;
        trans.get_extension(ext);

        if (!ext) {
            SC_REPORT_FATAL("TLM-2", "Missing extension in backward path!");
        }

        // Extract initiator id and route backwards
        int initiator_id = ext->initiator_id;
        return target_socket[initiator_id]->nb_transport_bw(trans, phase, delay);
    }

    SC_CTOR(Interconnect) {
        for (int i = 0; i < 2; i++) {
            char txt[20];
            sprintf(txt, "target_socket_%d", i);
            target_socket[i].register_nb_transport_fw(this, &Interconnect::nb_transport_fw, i);
            initiator_socket[i].register_nb_transport_bw(this, &Interconnect::nb_transport_bw, i);
        }
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
    int id;

    virtual tlm_sync_enum nb_transport_fw(tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay) {
        unsigned int len = trans.get_data_length();
        unsigned char* byt = trans.get_byte_enable_ptr();
        unsigned int wid = trans.get_streaming_width();

        if (byt != 0) { 
            trans.set_response_status(TLM_BYTE_ENABLE_ERROR_RESPONSE); 
            return TLM_COMPLETED; 
        }
        if (len > 4 || wid < len) { 
            trans.set_response_status(TLM_BURST_ERROR_RESPONSE); 
            return TLM_COMPLETED; 
        }

        m_peq.notify(trans, phase, delay);
        return TLM_ACCEPTED;
    }

    void peq_cb(tlm_generic_payload& trans, const tlm_phase& phase) {
        tlm_sync_enum status;

        switch (phase) {
            case BEGIN_REQ:
                trans.acquire();

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
                if (!response_in_progress) {
                    SC_REPORT_FATAL("TLM-2", "Illegal END_RESP");
                }

                cout << "@" << sc_time_stamp() << " Target " << id << ": Received END_RESP for 0x" << hex << trans.get_address() << dec << ". Closing transaction" << endl << endl;
                trans.release();
                n_trans--;

                response_in_progress = false;
                if (next_response_pending) {
                    send_response(*next_response_pending);
                    next_response_pending = 0;
                }
                if (end_req_pending) {
                    status = send_end_req(*end_req_pending);
                    end_req_pending = 0;
                }
                break;

            case END_REQ:
            case BEGIN_RESP:
                SC_REPORT_FATAL("TLM-2", "Illegal phase received by target");
                break;

            default:
                if (phase == internal_ph) {
                    cout << "@" << sc_time_stamp() << " Target " << id << ": Performing memory operation for 0x" << hex << trans.get_address() << dec << endl << endl;
                    
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

                    if (response_in_progress) {
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

    tlm_sync_enum send_end_req(tlm_generic_payload& trans) {
        tlm_phase bw_phase = END_REQ;
        sc_time delay = sc_time(10, SC_NS);
        
        cout << "@" << sc_time_stamp() << " Target " << id << ": Accepted request for 0x" << hex << trans.get_address() << dec << ". Sending END_REQ" << endl << endl;
        tlm_sync_enum status = socket->nb_transport_bw(trans, bw_phase, delay);
        
        if (status == TLM_COMPLETED) { 
            trans.release(); 
            return status; 
        }
        
        m_peq.notify(trans, internal_ph, delay + sc_time(20, SC_NS));
        n_trans++;
        return status;
    }

    void send_response(tlm_generic_payload& trans) {
        tlm_phase bw_phase = BEGIN_RESP;
        sc_time delay = SC_ZERO_TIME;
        
        cout << "@" << sc_time_stamp() << " Target " << id << ": Sending BEGIN_RESP for 0x" << hex << trans.get_address() << dec << endl << endl;
        response_in_progress = true;
        socket->nb_transport_bw(trans, bw_phase, delay);
    }

    SC_HAS_PROCESS(Target);
    Target(sc_module_name name, int _id) : sc_module(name), socket("socket"), m_peq(this, &Target::peq_cb), id(_id) {
        socket.register_nb_transport_fw(this, &Target::nb_transport_fw);
        for (int i = 0; i < SIZE; i++) {
            mem[i] = 0;
        }
        n_trans = 0; response_in_progress = false; next_response_pending = 0; end_req_pending = 0;
    }
};

int sc_main(int argc, char* argv[]) {
    // Initiator 0 writes to Target 0
    Initiator init0("init0", 0, 0x10);
    
    // Initiator 1 writes to Target 1
    Initiator init1("init1", 1, 0x110);
    
    Interconnect Interconnect("Interconnect");
    Target target0("target0", 0);
    Target target1("target1", 1);

    // Bind Initiators to Interconnect
    init0.socket.bind(Interconnect.target_socket[0]);
    init1.socket.bind(Interconnect.target_socket[1]);

    // Bind Interconnect to Targets
    Interconnect.initiator_socket[0].bind(target0.socket);
    Interconnect.initiator_socket[1].bind(target1.socket);

    sc_start();
    return 0;
}