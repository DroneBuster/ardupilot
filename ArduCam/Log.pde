#include <DataFlash.h>

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

enum log_messages {
    LOG_PERFORMANCE_MSG,
    LOG_STARTUP_MSG,
    LOG_RC_MSG
};

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
};

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance),
          "PM",  "IHI", "LTime,MLC,gDt" }
};

static uint16_t log_num;

// Read the DataFlash.log memory : Packet Parser
static void Log_Read(uint16_t log_num1, int16_t start_page, int16_t end_page)
{
    hal.console->printf_P(PSTR("\n" THISFIRMWARE "\nFree RAM: %u\n"),
                        (unsigned)hal.util->available_memory());

    hal.console->println("ArduPilot");

    DataFlash.LogReadProcess(log_num1, start_page, end_page,
                             NULL, hal.console);
}

static bool print_log_menu(void)
{
   // hal.console->println("Log menu");
    hal.console->println("logs enabled: ");
    hal.console->println("none");
    hal.console->println();
    DataFlash.ListAvailableLogs(hal.console);
    return true;
}

static int8_t dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(hal.console);
        return(-1);
    } else if (dump_log <= 0) {
        hal.console->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2)
               || ((uint16_t)dump_log > last_log_num))
    {
        hal.console->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return 0;
}

static void do_erase_logs(void)
{
    hal.console->println("Erasing logs");
    DataFlash.EraseAll();
    hal.console->println("Log erase complete");
}

static int8_t erase_logs(uint8_t argc, const Menu::arg *argv)
{
    do_erase_logs();
    return 0;
}

static void log_init(void) {
    DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));

    DataFlash.ReadManufacturerID();
    hal.scheduler->delay(10);
    DataFlash.ShowDeviceInfo(hal.console);

    if (DataFlash.NeedErase()) {
        do_erase_logs();
    }

    log_num = DataFlash.StartNewLog();
    hal.console->printf("Using log number %u\n", log_num);
}

// Write a performance monitoring packet. Total length : ? bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        loop_time       : delta_us_fast_loop,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}


