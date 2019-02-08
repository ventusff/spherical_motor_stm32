# ULog notes

more information can be found at: https://www.rt-thread.org/document/site/programming-manual/ulog/ulog/

 - LOG_TAG is default output log tag, when no tag given.
 - LOG_LVL is default output log level, when no level given.

 - LOG_X belongs with default output tag/module. defined by LOG_TAG
 - ulog_x can set different tags.


 - 全局静态日志级别：在 menuconfig 中配置，对应 ULOG_OUTPUT_LVL 宏。
 - 全局动态日志级别：使用 void ulog_global_filter_lvl_set(rt_uint32_t level) 函数来设定。
 - 模块静态日志级别：在模块（文件）内定义 LOG_LVL 宏，与日志标签宏 LOG_TAG 定义方式类似。
 - 模块动态日志级别：使用 int ulog_tag_lvl_filter_set(const char *tag, rt_uint32_t level) 函数来设定。

## about 模块动态日志：
because of this convinience, <br> we can use a seperate log tag/level for a vertain .c file.

```
/*
 * output different level log by LOG_X API
 *
 * NOTE: The `LOG_TAG` and `LOG_LVL` must be defined before including the <ulog.h> when you want to use LOG_X API.
 *
 * #define LOG_TAG              "example"
 * #define LOG_LVL              LOG_LVL_DBG
 * #include <ulog.h>
 *
 * Then you can using LOG_X API to output log
 *
 * LOG_D("this is a debug log!");
 * LOG_E("this is a error log!");
 */
#define LOG_E(...)                     ulog_e(LOG_TAG, __VA_ARGS__)
#define LOG_W(...)                     ulog_w(LOG_TAG, __VA_ARGS__)
#define LOG_I(...)                     ulog_i(LOG_TAG, __VA_ARGS__)
#define LOG_D(...)                     ulog_d(LOG_TAG, __VA_ARGS__)
#define LOG_RAW(...)                   ulog_raw(__VA_ARGS__)
#define LOG_HEX(name, width, buf, size)      ulog_hex(name, width, buf, size)
```