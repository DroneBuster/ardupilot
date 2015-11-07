#include <AP_Menu.h>

static const struct Menu::command main_menu_commands[] PROGMEM = {
//   command        function called
//   =======        ===============
    {"logs",                process_logs},
};

MENU(main_menu, THISFIRMWARE, main_menu_commands);

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);
    Menu::set_port(hal.console);
    hal.console->set_blocking_writes(true);

    while (1) {
        main_menu.run();
    }
}
