
#include <rtthread.h>

#ifdef RT_USING_LOGTRACE

#include <log_trace.h>
#include <string.h>
const static struct log_trace_session _main_session =
{
	.id = {.name = "main"},
	.lvl = LOG_TRACE_LEVEL_DEBUG,
};

void log_trace_example(void)
{
    /* log trace test */
    // log_trace_set_device("uart2");
    log_trace_register_session(&_main_session);
    while(1)
    {
        log_session(&_main_session, "testing...\n",LOG_TRACE_LEVEL_INFO);
        rt_thread_delay(DELAY_S(5));
    }
}

#endif