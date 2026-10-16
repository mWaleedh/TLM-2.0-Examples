#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/tlm_quantumkeeper.h>

using namespace std;
using namespace tlm;
using namespace tlm_utils;
using namespace sc_dt;

// Memory manager for initiator 2
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

// Initiator 1: b_transport, DMI, Debug
struct Initiator1 : sc_module {
    simple_initiator_socket<Initiator1> socket;
    
    tlm_quantumkeeper m_qk;
    tlm_dmi dmi_data;
    bool dmi_ptr_valid;

    void initiator_thread() {
        tlm_generic_payload trans;
        
        // Sync local time to global time
        m_qk.reset();
        sc_time delay = m_qk.get_local_time();

        // Debug write
        int debug_data = 0x12345678;
        trans.set_command(TLM_WRITE_COMMAND);
        trans.set_address(0x20);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&debug_data));
        trans.set_data_length(4);
        
        unsigned int n_bytes = socket->transport_dbg(trans);
        if (n_bytes == 4) {
            cout << "@" << sc_time_stamp() << " Initiator-1 Debug WRITE: Data = 0x12345678 | Global Address = 0x20 (Target 1)" << endl << endl;
        }

        // Write to Target 0
        int data0 = 0xAAAA;
        trans.set_command(TLM_WRITE_COMMAND);
        trans.set_address(0x10);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&data0));
        trans.set_data_length(4);
        trans.set_streaming_width(4);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        trans.set_response_status(TLM_INCOMPLETE_RESPONSE);
        
        socket->b_transport(trans, delay);
        cout << "@" << sc_time_stamp() << " Initiator-1 Blocking WRITE: Data = 0x" << hex << data0 << dec << " | Global Address = 0x10 (Target 1)" << endl << endl;

        // Annotate delay to Quantum Keeper instead of wait
        m_qk.set(delay);
        if (m_qk.need_sync()) {
            m_qk.sync();
        }

        // DMI read/write from Target 0
        if (trans.is_dmi_allowed()) {            
            if (socket->get_direct_mem_ptr(trans, dmi_data)) {
                dmi_ptr_valid = true;
                uint64 dmi_addr = 0x14;

                // Make sure the address is inside the boundaries
                if (dmi_addr >= dmi_data.get_start_address() && dmi_addr <= dmi_data.get_end_address()) {
                    
                    // Calculate memory location
                    unsigned char* dmi_ptr = dmi_data.get_dmi_ptr() + (dmi_addr - dmi_data.get_start_address());
                    
                    // Write
                    int write_data = 0xABCDEF12;
                    *(reinterpret_cast<int*>(dmi_ptr)) = write_data;
                    
                    // Add latency to the Quantum Keeper
                    m_qk.inc(dmi_data.get_write_latency());
                    
                    cout << "@" << sc_time_stamp() << " Initiator-1 DMI WRITE: Data = 0x" << hex << write_data << dec << " | Global Address = 0x" << hex << dmi_addr << dec << " (Target 1)" << endl << endl;

                    // Sync local time to global if time limit exceeded
                    if (m_qk.need_sync()) {
                        m_qk.sync();
                    }
                }
            }
        }
    }

    virtual void invalidate_direct_mem_ptr(uint64 start_range, uint64 end_range) {
        dmi_ptr_valid = false;
        cout << "Initiator-1 Received DMI invalidation for global range 0x" << hex << start_range << " to 0x" << end_range << endl << endl;
    }

    SC_CTOR(Initiator1) : socket("socket"), dmi_ptr_valid(false) {
        SC_THREAD(initiator_thread);
        socket.register_invalidate_direct_mem_ptr(this, &Initiator1::invalidate_direct_mem_ptr);
    }
};

// Initiator 2: b_transport, Memory Manager
struct Initiator2 : sc_module {
    simple_initiator_socket<Initiator2> socket;
    
    tlm_quantumkeeper m_qk;
    MemoryManager m_mm;

    void initiator_thread() {
        m_qk.reset();
        
        // Loop multiple transactions without waiting for Temporal Decoupling and Memory manager working
        for (int i = 0x110; i <= 0x11C; i += 4) {
            sc_time delay = m_qk.get_local_time();

            // Allocate generic payload from the memory manager pool
            tlm_generic_payload* trans = m_mm.allocate();
            trans->acquire();   // Increment reference count

            int* data1 = new int(i);    // Dynamic allocation for payload pointer
            trans->set_command(TLM_WRITE_COMMAND);
            trans->set_address(i); 
            trans->set_data_ptr(reinterpret_cast<unsigned char*>(data1));
            trans->set_data_length(4);
            trans->set_streaming_width(4);
            trans->set_byte_enable_ptr(nullptr);
            trans->set_dmi_allowed(false);
            trans->set_response_status(TLM_INCOMPLETE_RESPONSE);

            socket->b_transport(*trans, delay);
            cout << "@" << sc_time_stamp() << " Initiator-2 Blocking WRITE: Data = 0x" << hex << *data1 << dec << " | Global Address = 0x" << hex << i << dec << " (Target 2)" << endl << endl;

            // Annotate delay to Quantum Keeper
            m_qk.set(delay);
            if (m_qk.need_sync()) {
                m_qk.sync();
            }

            // Release payload back to memory manager pool
            trans->release();
            delete data1;
        }
    }

    SC_CTOR(Initiator2) : socket("socket") {
        SC_THREAD(initiator_thread);
    }
};

struct Interconnect : sc_module {
    // Array of tagged target sockets to handle multiple Initiators
    simple_target_socket_tagged<Interconnect> target_socket[2];
    
    // Array of tagged initiator sockets for backward callback
    simple_initiator_socket_tagged<Interconnect> initiator_socket[2];

    virtual void b_transport(int id, tlm_generic_payload& trans, sc_time& delay) {
        uint64 global_addr = trans.get_address();
        int target_id;
        uint64 target_base;

        // Decode address
        if (global_addr <= 0xFF) {
            target_id = 0;          // Route to Target 0
            target_base = 0x0;
        } 
        else if (global_addr >= 0x100 && global_addr <= 0x1FF) {
            target_id = 1;          // Route to Target 1
            target_base = 0x100;
        } 
        else {
            // Incorrect address
            trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }

        // Forward the transaction to correct target with the local address
        trans.set_address(global_addr - target_base);
        initiator_socket[target_id]->b_transport(trans, delay);

        // Restore the global address before returning
        trans.set_address(global_addr);
    }

    virtual unsigned int transport_dbg(int id, tlm_generic_payload& trans) {
        uint64 global_addr = trans.get_address();
        int target_id;
        uint64 target_base;

        // Decode address
        if (global_addr <= 0xFF) { 
            target_id = 0; target_base = 0x0; 
        } 
        else if (global_addr >= 0x100 && global_addr <= 0x1FF) { 
            target_id = 1; target_base = 0x100; 
        } 
        else { 
            return 0; 
        }
        
        // Calculate local address
        trans.set_address(global_addr - target_base);

        // Forward to target
        unsigned int bytes_processed = initiator_socket[target_id]->transport_dbg(trans);

        // Reset to global address
        trans.set_address(global_addr);

        return bytes_processed;
    }

    virtual bool get_direct_mem_ptr(int id, tlm_generic_payload& trans, tlm_dmi& dmi_data) {
        uint64 global_addr = trans.get_address();
        int target_id;
        uint64 target_base;

        // Decode address
        if (global_addr <= 0xFF) { 
            target_id = 0; target_base = 0x0; 
        }
        else if (global_addr >= 0x100 && global_addr <= 0x1FF) { 
            target_id = 1; target_base = 0x100; 
        }
        else { 
            return false; 
        }

        // Calculate local address and forward to target
        trans.set_address(global_addr - target_base);
        bool status = initiator_socket[target_id]->get_direct_mem_ptr(trans, dmi_data);
        
        // Adjust the DMI boundaries to global addresses for Initiator
        if (status) {
            dmi_data.set_start_address(dmi_data.get_start_address() + target_base);
            dmi_data.set_end_address(dmi_data.get_end_address() + target_base);
        }

        trans.set_address(global_addr);
        return status;
    }

    virtual void invalidate_direct_mem_ptr(int id, uint64 start_range, uint64 end_range) {
        uint64 target_base;

        // Get global base address
        if (id == 0) { 
            target_base = 0x0; 
        } 
        else if (id == 1) { 
            target_base = 0x100; 
        } 
        else { 
            return; 
        }

        // Calculate global address
        uint64 global_start = start_range + target_base;
        uint64 global_end = end_range + target_base;

        // Forward the translated invalidation backward to BOTH CPUs
        for (int i = 0; i < 2; i++) {
            target_socket[i]->invalidate_direct_mem_ptr(global_start, global_end);
        }
    }

    SC_CTOR(Interconnect) {
        // Register tagged forward callbacks for both target sockets
        for (int i = 0; i < 2; i++) {
            char txt[20];
            sprintf(txt, "target_socket_%d", i);
            target_socket[i].register_b_transport(this, &Interconnect::b_transport, i);
            target_socket[i].register_transport_dbg(this, &Interconnect::transport_dbg, i);
            target_socket[i].register_get_direct_mem_ptr(this, &Interconnect::get_direct_mem_ptr, i);
            
            // Register tagged backward callback for both initiator sockets
            initiator_socket[i].register_invalidate_direct_mem_ptr(this, &Interconnect::invalidate_direct_mem_ptr, i);
        }
    }
};

// Target unchanged from previous lectures
struct Target : sc_module {
    simple_target_socket<Target> socket;

    // 64 * 4 bytes = 0x100 or 256
    static constexpr int SIZE = 64;
    int mem[SIZE];

    // Implement b_transport method for Standard Blocking Calls
    virtual void b_transport(tlm_generic_payload& trans, sc_time& delay) {
        tlm_command cmd = trans.get_command();
        uint64 addr = trans.get_address() / 4;
        unsigned char* ptr = trans.get_data_ptr();
        unsigned int len = trans.get_data_length();
        unsigned char* byte = trans.get_byte_enable_ptr();
        unsigned int wid = trans.get_streaming_width();

        // Specific Error Checking
        if (addr >= SIZE) {
            trans.set_response_status(TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }
        if (byte != 0) {
            trans.set_response_status(TLM_BYTE_ENABLE_ERROR_RESPONSE);
            return;
        }
        if (len > 4 || wid < len) {
            trans.set_response_status(TLM_BURST_ERROR_RESPONSE);
            return;
        }

        // Perform read or write
        if (cmd == TLM_READ_COMMAND) {
            memcpy(ptr, &mem[addr], len);
        } 
        else if (cmd == TLM_WRITE_COMMAND) {
            memcpy(&mem[addr], ptr, len);
        } 
        else {
            trans.set_response_status(TLM_COMMAND_ERROR_RESPONSE);
            return;
        }

        // Hint initiator that DMI is allowed
        trans.set_dmi_allowed(true);

        // Annotate processing delay
        delay += sc_time(10, SC_NS);

        // Set successful response status
        trans.set_response_status(TLM_OK_RESPONSE);
    }

    // Implement transport_dbg
    virtual unsigned int transport_dbg(tlm_generic_payload& trans) {
        tlm_command cmd = trans.get_command();
        
        uint64 addr = trans.get_address() / 4; 
        unsigned char* ptr = trans.get_data_ptr();
        unsigned int len = trans.get_data_length();

        // Boundary check
        if (addr >= SIZE) {
            return 0; 
        }

        // Perform read/write
        if (cmd == TLM_READ_COMMAND) {
            memcpy(ptr, &mem[addr], len);
        } 
        else if (cmd == TLM_WRITE_COMMAND) {
            memcpy(&mem[addr], ptr, len);
        }

        // Return number of bytes processed
        return len;
    }

    // Implement get_direct_mem_ptr method for Direct Memory Interface
    virtual bool get_direct_mem_ptr(tlm_generic_payload& trans, tlm_dmi& dmi_data) {
        dmi_data.allow_read_write();
        dmi_data.set_dmi_ptr(reinterpret_cast<unsigned char*>(&mem[0]));    // Byte addressable pointer
        dmi_data.set_start_address(0);
        dmi_data.set_end_address((SIZE * 4) - 1);
        dmi_data.set_read_latency(sc_time(1, SC_NS));
        dmi_data.set_write_latency(sc_time(1, SC_NS));

        // Return true, signalling DMI is allowed
        return true;
    }

    void memory_lock() {
        cout << "Triggering DMI Lock. Revoking pointers." << endl;

        socket->invalidate_direct_mem_ptr(0, (SIZE * 4) - 1);
    }

    SC_CTOR(Target) : socket("socket") {
        // Register the three interfaces.
        socket.register_b_transport(this, &Target::b_transport);
        socket.register_transport_dbg(this, &Target::transport_dbg);
        socket.register_get_direct_mem_ptr(this, &Target::get_direct_mem_ptr);

        // Initialize memory
        for (int i = 0; i < SIZE; i++) {
            mem[i] = 0;
        }
    }
};

int sc_main(int argc, char* argv[]) {
    Initiator1 init1("init1");
    Initiator2 init2("init2");
    Interconnect Interconnect("Interconnect");
    Target target0("target0");
    Target target1("target1");

    // Initiators to Interconnect
    init1.socket.bind(Interconnect.target_socket[0]);
    init2.socket.bind(Interconnect.target_socket[1]);

    // Interconnect to Targets
    Interconnect.initiator_socket[0].bind(target0.socket);
    Interconnect.initiator_socket[1].bind(target1.socket);

    // Set the Global Quantum limit for the whole system
    tlm_global_quantum::instance().set(sc_time(25, SC_NS));

    sc_start();
    return 0;
}