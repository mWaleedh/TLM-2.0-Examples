#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

using namespace std;
using namespace tlm;
using namespace tlm_utils;
using namespace sc_dt;

// Initiator module
struct Initiator : sc_module {
    // initiator socket
    simple_initiator_socket<Initiator> socket;    

    void initiator_thread() {
        tlm_generic_payload trans;
        sc_time delay = SC_ZERO_TIME;

        // Write data
        for (int i = 32; i < 96; i += 4) {
            // Value to write
            int data = i;

            // 8 required attributes, remaining 2 are optional
            trans.set_command(TLM_WRITE_COMMAND);   // Set command to WRITE
            trans.set_address(i);
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(&data));
            trans.set_data_length(4);
            trans.set_streaming_width(4);   // must be <= data_length, normally kept equal
            trans.set_byte_enable_ptr(0);
            trans.set_dmi_allowed(false);
            trans.set_response_status(TLM_INCOMPLETE_RESPONSE); // Always initialized to INCOMPLETE

            socket->b_transport(trans, delay);

            // Check for error
            if (trans.is_response_error()) {
                SC_REPORT_ERROR("TLM-2", "Response error from b_transport");
            }

            cout << "@" << sc_time_stamp() << " WRITE: Address = 0x" << hex << i << " | Data = 0x" << data << dec << " | Delay = " << delay << endl << endl;

            // Annotate delay and reset to 0
            wait(delay);
            delay = SC_ZERO_TIME;
        }

        // Read back data
        for (int i = 32; i < 96; i += 4) {
            // value to read
            int data;

            // 8 required attributes, remaining 2 are optional
            trans.set_command(TLM_READ_COMMAND);    // Set command to READ
            trans.set_address(i);
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(&data));
            trans.set_data_length(4);
            trans.set_streaming_width(4);   // must be <= data_length, normally kept equal
            trans.set_byte_enable_ptr(0);
            trans.set_dmi_allowed(false);
            trans.set_response_status(TLM_INCOMPLETE_RESPONSE); // Always initialized to INCOMPLETE

            socket->b_transport(trans, delay);

            // Check for error
            if (trans.is_response_error()) {
                SC_REPORT_ERROR("TLM-2", "Response error from b_transport");
            }

            cout << "@" << sc_time_stamp() << " READ: Address = 0x" << hex << i << " | Data = 0x" << data << dec << " | Delay = " << delay << endl << endl;

            // Annotate delay and reset to 0
            wait(delay);
            delay = SC_ZERO_TIME;
        }
    }

    SC_CTOR(Initiator) : socket("socket") {
        SC_THREAD(initiator_thread);
    }
};

// Target module
struct Target : sc_module {
    // target socket
    simple_target_socket<Target> socket;

    static constexpr int SIZE = 256;
    int mem[SIZE];

    // Implement b_transport method for Standard Blocking Calls
    virtual void b_transport(tlm_generic_payload& trans, sc_time& delay) {
        tlm_command cmd = trans.get_command();
        uint64 addr = trans.get_address() / 4;
        unsigned char* ptr = trans.get_data_ptr();
        unsigned int len = trans.get_data_length();
        unsigned char* byte = trans.get_byte_enable_ptr();
        unsigned int wid = trans.get_streaming_width();

        // End transaction if error
        if (addr >= uint64(SIZE) || byte != 0 || len > 4 || wid < len) {
            SC_REPORT_ERROR("TLM-2", "Target does not support given generic payload transaction");
        }

        // Perform read or write
        if (cmd == TLM_READ_COMMAND) {
            memcpy(ptr, &mem[addr], len);
        } 
        else if (cmd == TLM_WRITE_COMMAND) {
            memcpy(&mem[addr], ptr, len);
        }

        // Annotate processing delay
        delay += sc_time(10, SC_NS);

        // Set successful response status
        trans.set_response_status(TLM_OK_RESPONSE);
    }

    SC_CTOR(Target) : socket("socket") {
        // Register call back method
        socket.register_b_transport(this, &Target::b_transport);

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