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

        // Debug transport transaction, is performed in zero simulation time
        int debug_val = 0xABCDEF12;
        trans.set_command(TLM_WRITE_COMMAND);
        trans.set_address(0x10);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&debug_val));
        trans.set_data_length(4);

        unsigned int bytes_written = socket->transport_dbg(trans);
        cout << "@" << sc_time_stamp() << " Debug WRITE: Bytes Written = " << bytes_written << " | Data = 0xabcdef12 | Address = 0x10 | Delay = 0 ns" << endl << endl;

        // Normal b_transport transaction
        int read_val = 0;
        trans.set_command(TLM_READ_COMMAND);
        trans.set_address(0x10);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&read_val));
        trans.set_data_length(4);
        trans.set_streaming_width(4);
        trans.set_byte_enable_ptr(0);
        trans.set_dmi_allowed(false);   // Set to false by default
        trans.set_response_status(TLM_INCOMPLETE_RESPONSE);

        socket->b_transport(trans, delay);

        // Check for error and print specific error
        if (trans.is_response_error()) {
            SC_REPORT_ERROR("TLM-2", trans.get_response_string().c_str());
        }

        cout << "@" << sc_time_stamp() << " Blocking READ: Data = 0x" << hex << read_val << dec << " | Address = 0x10" << " | Delay = " << delay << endl << endl;

        wait(delay);
        delay = SC_ZERO_TIME;

        // DMI transaction
        if (trans.is_dmi_allowed()) {
            // Check if DMI is allowed
            bool success = socket->get_direct_mem_ptr(trans, dmi_data);

            if (success) {
                // Get memory pointer
                unsigned char* dmi_ptr = dmi_data.get_dmi_ptr();

                // Direct write
                int dmi_write = 0x11223344;
                uint64 target_addr = 0x20;
                memcpy(dmi_ptr + target_addr, &dmi_write, 4);
                delay += dmi_data.get_write_latency();

                cout << "@" << sc_time_stamp() << " DMI WRITE: Data = 0x" << hex << dmi_write << " | Address = 0x" << target_addr << dec << " | Delay = " << dmi_data.get_write_latency() << endl << endl;

                wait(delay);
                delay = SC_ZERO_TIME;

                // Direct read
                int dmi_read = 0;
                memcpy(&dmi_read, dmi_ptr + target_addr, 4);
                delay += dmi_data.get_read_latency();

                cout << "@" << sc_time_stamp() << " DMI READ: Data = 0x" << hex << dmi_read << " | Address = 0x" << target_addr << dec << " | Delay = " << dmi_data.get_read_latency() << endl << endl;

                wait(delay);
                delay = SC_ZERO_TIME;
            }
        }

        // Out of bounds error handling
        int dummy = 0;
        trans.set_command(TLM_READ_COMMAND);
        trans.set_address(0xFFFF);  // Out of bounds address
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&dummy));
        trans.set_data_length(4);
        trans.set_streaming_width(4);
        trans.set_byte_enable_ptr(0);
        trans.set_dmi_allowed(false);
        trans.set_response_status(TLM_INCOMPLETE_RESPONSE);

        socket->b_transport(trans, delay);

        // Check for error
        if (trans.is_response_error()) {
            cout << "Caught error: " << trans.get_response_string() << endl;
            cout << "Response status: " << trans.get_response_status() << endl << endl;
        } 
        else {
            cout << "ERROR: Out of bounds transaction didn't give error" << endl << endl;
        }
    }

    SC_CTOR(Initiator) : socket("socket") {
        SC_THREAD(initiator_thread);
    }
};

struct Target : sc_module {
    simple_target_socket<Target> socket;

    static constexpr int SIZE = 256;
    int mem[SIZE];

    // Implement b_transport method for standard blocking aalls
    virtual void b_transport(tlm_generic_payload& trans, sc_time& delay) {
        tlm_command cmd = trans.get_command();
        uint64 addr = trans.get_address() / 4;
        unsigned char* ptr = trans.get_data_ptr();
        unsigned int len = trans.get_data_length();
        unsigned char* byte = trans.get_byte_enable_ptr();
        unsigned int wid = trans.get_streaming_width();

        // Specific error checking
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

    // Implement get_direct_mem_ptr method for Direct Memory Interface
    virtual bool get_direct_mem_ptr(tlm_generic_payload& trans, tlm_dmi& dmi_data) {
        dmi_data.allow_read_write();
        dmi_data.set_dmi_ptr(reinterpret_cast<unsigned char*>(&mem[0]));    // Byte addressable pointer
        dmi_data.set_start_address(0);
        dmi_data.set_end_address(SIZE * 4 - 1);
        dmi_data.set_read_latency(sc_time(1, SC_NS));
        dmi_data.set_write_latency(sc_time(1, SC_NS));

        // Return true, signalling DMI is allowed
        return true;
    }

    // Implement transport_dbg method for Debug Transport Interface
    virtual unsigned int transport_dbg(tlm_generic_payload& trans) {
        tlm_command cmd = trans.get_command();
        uint64 addr = trans.get_address();
        unsigned char* ptr = trans.get_data_ptr();
        unsigned int len = trans.get_data_length();

        // Check for out of bounds access
        if (addr >= SIZE * 4) {
            return 0;
        }

        // Calculate total bytes being copied
        unsigned int num_bytes = (len > (SIZE * 4 - addr)) ? (SIZE * 4 - addr) : len;

        // Byte addressable pointer to memory
        unsigned char* mem_bytes = reinterpret_cast<unsigned char*>(&mem[0]);

        // Perform read or write
        if (cmd == TLM_READ_COMMAND) {
            memcpy(ptr, &mem_bytes[addr], num_bytes);
        } 
        else if (cmd == TLM_WRITE_COMMAND) {
            memcpy(&mem_bytes[addr], ptr, num_bytes);
        }

        // Return actual bytes copied (required by design)
        return num_bytes;
    }

    SC_CTOR(Target) : socket("socket") {
        // Register all three interfaces.
        // Otherwise simple socket will use generic implementations that do nothing
        socket.register_b_transport(this, &Target::b_transport);
        socket.register_get_direct_mem_ptr(this, &Target::get_direct_mem_ptr);
        socket.register_transport_dbg(this, &Target::transport_dbg);

        // Initialize memory
        for (int i = 0; i < SIZE; i++) {
            mem[i] = 0;
        }
    }
};

int sc_main(int argc, char* argv[]) {
    Initiator initiator("initiator");
    Target target("target");

    // Bind sockets
    initiator.socket.bind(target.socket);

    sc_start();
    return 0;
}