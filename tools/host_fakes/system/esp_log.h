#ifndef HOST_FAKE_LOG_H
#define HOST_FAKE_LOG_H
/* 实参仍求值，使真实驱动日志变量受到编译检查，但主机测试不打印硬件日志。 */
static inline void host_fake_log(const char *tag, const char *format, ...) {
    (void)tag; (void)format;
}
const char *esp_err_to_name(int error);
#define ESP_LOGI(...) host_fake_log(__VA_ARGS__)
#define ESP_LOGW(...) host_fake_log(__VA_ARGS__)
#define ESP_LOGE(...) host_fake_log(__VA_ARGS__)
#endif
