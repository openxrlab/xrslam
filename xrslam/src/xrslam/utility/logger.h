#include <string>
#include <vector>

namespace xrslam {
class Logger {
  public:
    static Logger &instance() {
        static Logger logger;
        return logger;
    }
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    void pushMessage(std::string msg) { this->message_list.emplace_back(msg); }

    std::vector<std::string> getAllMessage() {
        std::vector<std::string> ret(this->message_list);
        this->message_list.clear();
        return ret;
    }

  private:
    Logger() { this->message_list = std::vector<std::string>(); }
    ~Logger() = default;
    std::vector<std::string> message_list;
};
} // namespace xrslam
