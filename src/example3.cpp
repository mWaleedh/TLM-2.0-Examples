#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

using namespace std;
using namespace tlm;
using namespace tlm_utils;
using namespace sc_dt;

struct Initiator : sc_module {
    simple_initiator_socket<Initiator> socket;

    void initiator_thread() {
        tlm_generic_payload trans;
        sc_time delay = SC_ZERO_TIME;
        tlm_dmi dmi_data;

        // Test debug write
        int debug_data = 0x12345678;
        trans.set_command(TLM_WRITE_COMMAND);
        trans.set_address(0x20);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&debug_data));
        trans.set_data_length(4);
        
        // Call transport_dbg
        unsigned int n_bytes = socket->transport_dbg(trans);
        
        if (n_bytes == 4) {
            cout << "@" << sc_time_stamp() << " Debug WRITE: Data = 0x12345678 | Global Address = 0x20 (Target 1) | Delay = 0 ns" << endl << endl;
        } 
        else {
            cout << "@" << sc_time_stamp() << " Debug WRITE: ERROR. " << n_bytes << " bytes written" << endl << endl;
        }

        // Write to Target 0 (Address 0x0 - 0xFF)
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
        cout << "@" << sc_time_stamp() << " Blocking WRITE: Data = 0x" << hex << data0 << dec << " | Global Address = 0x10 (Target 1) | Delay = " << delay << endl << endl;

        wait(delay);
        delay = SC_ZERO_TIME;

        // Write to Target 1 (Address 0x100 - 0x1FF)
        int data1 = 0xBBBB;
        trans.set_command(TLM_WRITE_COMMAND);
        trans.set_address(0x115); 
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&data1));
        trans.set_data_length(4);
        trans.set_streaming_width(4);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        trans.set_response_status(TLM_INCOMPLETE_RESPONSE);

        socket->b_transport(trans, delay);
        cout << "@" << sc_time_stamp() << " Blocking WRITE: Data = 0x" << hex << data1 << dec << " | Global Address = 0x115 (Target 2) | Delay = " << delay << endl << endl;

        wait(delay);
        delay = SC_ZERO_TIME;

        // DMI read from Target 1
        if (trans.is_dmi_allowed()) {            
            if (socket->get_direct_mem_ptr(trans, dmi_data)) {
                uint64 dmi_addr = 0x115;

                // Make sure the address is inside the boundaries
                if (dmi_addr >= dmi_data.get_start_address() && dmi_addr <= dmi_data.get_end_address()) {
                    
                    // Calculate memory location
                    unsigned char* dmi_ptr = dmi_data.get_dmi_ptr() + (dmi_addr - dmi_data.get_start_address());
                    
                    // Read
                    int read_data = *(reinterpret_cast<int*>(dmi_ptr));
                    delay += dmi_data.get_read_latency();

                    cout << "@" << sc_time_stamp() << " DMI READ: Data = 0x" << hex << read_data << " | Global Address = 0x" << dmi_addr << dec << " (Target 2) | Delay = " << dmi_data.get_read_latency() << endl << endl;

                    wait(delay);
                    delay = SC_ZERO_TIME;

                    // Write
                    int write_data = 0xABCDEF12;
                    *(reinterpret_cast<int*>(dmi_ptr)) = write_data;
                    delay += dmi_data.get_write_latency();

                    cout << "@" << sc_time_stamp() << " DMI WRITE: Data = 0x" << hex << write_data << " | Global Address = 0x" << dmi_addr << dec << " (Target 2) | Delay = " << dmi_data.get_write_latency() << endl << endl;

                    wait(delay);
                    delay = SC_ZERO_TIME;
                }
            }
        }
    }

    virtual void invalidate_direct_mem_ptr(uint64 start_range, uint64 end_range) {
        // Nothing happens here for now
        cout << "Received DMI invalidation for global range 0x" << hex << start_range << " to 0x" << end_range << endl << endl;
    }

    SC_CTOR(Initiator) : socket("socket") {
        SC_THREAD(initiator_thread);

        // Register callback receiver
        socket.register_invalidate_direct_mem_ptr(this, &Initiator::invalidate_direct_mem_ptr);
    }
};

struct Interconnect : sc_module {
    // One target socket facing the CPU
    simple_target_socket<Interconnect> target_socket;
    
    // Array of tagged initiator sockets for backward callback
    simple_initiator_socket_tagged<Interconnect> initiator_socket[2];

    virtual void b_transport(tlm_generic_payload& trans, sc_time& delay) {
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

        // Restore the global address before returning to the Initiator (because same transaction object)
        trans.set_address(global_addr);
    }

    virtual unsigned int transport_dbg(tlm_generic_payload& trans) {
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
            return 0; 
        }

        // Translate to local address and forward
        trans.set_address(global_addr - target_base);
        unsigned int bytes_processed = initiator_socket[target_id]->transport_dbg(trans);

        // Restore the global address
        trans.set_address(global_addr);

        return bytes_processed;
    }

    virtual bool get_direct_mem_ptr(tlm_generic_payload& trans, tlm_dmi& dmi_data) {
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
            return false;
        }

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

        // Get base address
        if (id == 0) {
            target_base = 0x0;
        } 
        else if (id == 1) {
            target_base = 0x100;
        } 
        else {
            return;
        }

        // Translate the target's local boundaries to global boundaries
        uint64 global_start = start_range + target_base;
        uint64 global_end = end_range + target_base;

        cout << "Target " << id << " invalidated DMI. Forwarding global range 0x" << hex << global_start << " to 0x" << global_end << endl << endl;

        // Forward the translated invalidation backward to the CPU
        target_socket->invalidate_direct_mem_ptr(global_start, global_end);
    }

    SC_CTOR(Interconnect) : target_socket("target_socket") {
        target_socket.register_b_transport(this, &Interconnect::b_transport);
        target_socket.register_transport_dbg(this, &Interconnect::transport_dbg);
        target_socket.register_get_direct_mem_ptr(this, &Interconnect::get_direct_mem_ptr);

        // Register tagged backward callback
        for (int i = 0; i < 2; i++) {
            initiator_socket[i].register_invalidate_direct_mem_ptr(this, &Interconnect::invalidate_direct_mem_ptr, i);
        }
    }
};

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
    Initiator initiator("initiator");
    Interconnect Interconnect("Interconnect");
    Target target0("target0");
    Target target1("target1");

    // Initiator to interconnect
    initiator.socket.bind(Interconnect.target_socket);

    // Interconnect to the targets
    Interconnect.initiator_socket[0].bind(target0.socket);
    Interconnect.initiator_socket[1].bind(target1.socket);

    sc_start();
    return 0;
}