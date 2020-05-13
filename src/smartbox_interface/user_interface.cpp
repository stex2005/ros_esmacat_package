#include "user_interface.h"


using namespace  std;

int main(){

    esmacat_shared_memory_comm ecat_sm;
    ecat_sm.init();
    char c;
    string inputString;
    RobotState state(STOP);
    bool swap_state(false);


    print_command_keys();

    do{
        // Pull local state from shared memory
        state = ecat_sm.data->state;
        swap_state = ecat_sm.data->swap_state;

        // Get character
        c = cin.get();

        if(c != '\n'){
            switch(c){

            case 's': case 'S':
                if (state != STOP)
                {
                    std::cout << green_key << "Quick-swapped to STOP mode!" << color_key << std::endl;
                    state = STOP;
                    swap_state = true;
                }
                else
                {
                    std::cout << yellow_key << "Already in STOP mode" << color_key <<  std::endl;
                }
                break;
            case 'n': case 'N':
                if (state != NULLTORQUE)
                {
                    std::cout << green_key << "Quick-swapped to NULL-TORQUE mode!" << color_key << std::endl;
                    state = NULLTORQUE;
                    swap_state = true;
                }
                else
                {
                    std::cout << yellow_key << "Already in NULL-TORQUE mode" << color_key <<  std::endl;
                }
                break;
            case 'g': case 'G':
                if (state != GRAVITY)
                {
                    std::cout << green_key << "Quick-swapped to GRAVITY mode!" << color_key << std::endl;
                    state = GRAVITY;
                    swap_state = true;
                }
                else
                {
                    std::cout << yellow_key << "Already in GRAVITY mode" << color_key <<  std::endl;
                }
                break;
            case 'f': case 'F':
                if (state != FREEZE)
                {
                    std::cout << green_key << "Quick-swapped to FREEZE mode!" << color_key << std::endl;
                    state = FREEZE;
                    swap_state = true;
                }
                else
                {
                    std::cout << yellow_key << "Already in FREEZE mode" << color_key <<  std::endl;
                }
                break;
            case 'k': case 'K':

                if (ecat_sm.data->state == STOP or ecat_sm.data->state == EXIT)
                {   swap_state = true;
                    state = EXIT;
                  std::cout << yellow_key << "Ending program - no more inputs..." << color_key << std::endl;
                 }
                    else {
                  std::cout << yellow_key << "First stop the motor" << color_key << std::endl;

                 }
                break;
            case ' ':
                print_command_keys();
                break;
            default:
                std::cout << red_key << "Unrecognized key input!" << color_key <<  std::endl;
                break;
            }

            // Set local state to shared memory
            ecat_sm.data->state = state;
            ecat_sm.data->swap_state = swap_state;

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }
        else{
            std::cout << yellow_key << state_labels[state] << color_key << " is the state currently active" << std::endl;
        }


    }while(state != EXIT );
    return 0;
}

// Print commands on terminal
void print_command_keys()
{
    std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
    std::cout << blue_key << "\'k\'" << color_key << ": exit" << "\n";
    std::cout << blue_key << "\'s\'" << color_key << ": STOP mode"<< "\n";
    std::cout << blue_key << "\'n\'" << color_key << ": NULL-TORQUE mode" << "\n";
    std::cout << blue_key << "\'g\'" << color_key << ": GRAVITY mode"<< "\n";
    std::cout << blue_key << "\'f\'" << color_key << ": FREEZE mode"<< "\n";
    std::cout << blue_key << "\'ENTER\'" << color_key << ": SHOW current settings and command keys\n"<< "\n";

}

