#include "discocal_ros/performance_report.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <sys/resource.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <opencv2/core/version.hpp>
#include <ceres/version.h>

#include "discocal_ros/camera_info_writer.h"

namespace discocal_ros {
namespace {

double timeval_seconds(const struct timeval& tv) {
    return static_cast<double>(tv.tv_sec) + static_cast<double>(tv.tv_usec) * 1e-6;
}

CpuTimes rusage_cpu(int who) {
    CpuTimes out;
    struct rusage usage;
    std::memset(&usage, 0, sizeof(usage));
    if (getrusage(who, &usage) == 0) {
        out.user_seconds = timeval_seconds(usage.ru_utime);
        out.system_seconds = timeval_seconds(usage.ru_stime);
    }
    return out;
}

std::string iso8601_now() {
    const std::time_t now = std::time(NULL);
    struct tm local;
    std::memset(&local, 0, sizeof(local));
    localtime_r(&now, &local);
    char buffer[64];
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &local) == 0) {
        return "";
    }
    return buffer;
}

long peak_rss_kb() {
    struct rusage usage;
    std::memset(&usage, 0, sizeof(usage));
    if (getrusage(RUSAGE_SELF, &usage) != 0) return 0;
    // Linux reports ru_maxrss in kilobytes. (BSD/macOS use bytes; this package
    // targets Linux/ROS Melodic.)
    return static_cast<long>(usage.ru_maxrss);
}

int read_thread_count() {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.compare(0, 8, "Threads:") == 0) {
            return std::atoi(line.c_str() + 8);
        }
    }
    return 0;
}

std::string yaml_quote(const std::string& value) {
    std::string out = "\"";
    for (std::size_t i = 0; i < value.size(); ++i) {
        const char c = value[i];
        if (c == '\\' || c == '"') out += '\\';
        out += c;
    }
    out += '"';
    return out;
}

std::string number(double value, int precision = 6) {
    std::ostringstream oss;
    oss << std::setprecision(precision) << value;
    return oss.str();
}

void write_stage(std::ostringstream& out, const std::string& indent, const std::string& key,
                 const StageTiming& stage) {
    out << indent << key << ":\n";
    out << indent << "  ran: " << (stage.ran ? "true" : "false") << "\n";
    out << indent << "  wall_seconds: " << number(stage.wall_seconds) << "\n";
    out << indent << "  cpu_user_seconds: " << number(stage.cpu_user_seconds) << "\n";
    out << indent << "  cpu_system_seconds: " << number(stage.cpu_system_seconds) << "\n";
}

double stage_cpu(const StageTiming& stage) {
    return stage.cpu_user_seconds + stage.cpu_system_seconds;
}

}  // namespace

CpuTimes process_cpu_times() { return rusage_cpu(RUSAGE_SELF); }

CpuTimes thread_cpu_times() { return rusage_cpu(RUSAGE_THREAD); }

int process_thread_count() { return read_thread_count(); }

StageClock::StageClock() : wall_start_(std::chrono::steady_clock::now()),
                           cpu_start_(thread_cpu_times()) {}

StageTiming StageClock::stop() {
    StageTiming out;
    if (stopped_) return out;
    stopped_ = true;
    const CpuTimes cpu_end = thread_cpu_times();
    out.ran = true;
    out.wall_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start_).count();
    out.cpu_user_seconds = cpu_end.user_seconds - cpu_start_.user_seconds;
    out.cpu_system_seconds = cpu_end.system_seconds - cpu_start_.system_seconds;
    // Clamp tiny negative deltas from getrusage rounding.
    if (out.cpu_user_seconds < 0.0) out.cpu_user_seconds = 0.0;
    if (out.cpu_system_seconds < 0.0) out.cpu_system_seconds = 0.0;
    return out;
}

void CollectionCounters::add(const CollectionCounters& other) {
    frames_decoded += other.frames_decoded;
    frames_detected += other.frames_detected;
    frames_accepted += other.frames_accepted;
    frames_duplicate += other.frames_duplicate;
    frames_unpaired += other.frames_unpaired;
}

PerformanceReport::PerformanceReport()
    : wall_start_(std::chrono::steady_clock::now()) {}

void PerformanceReport::begin(const std::string& camera, int budget) {
    started_ = true;
    finished_ = false;
    wall_start_ = std::chrono::steady_clock::now();
    process_cpu_start_ = process_cpu_times();
    started_at_text_ = iso8601_now();
    camera_ = camera;
    budget_ = budget;
    attempts_ = 0;
    success_ = false;
    message_.clear();
    collection_ = CollectionCounters();
    collection_timing_ = StageTiming();
    timings_ = RigTimings();
    dwecal_ = DwecalTiming();
    process_cpu_end_ = CpuTimes();
    peak_rss_kb_ = 0;
    threads_peak_ = 0;
    warnings_.clear();
    sample_threads();
}

void PerformanceReport::sample_threads() {
    const int threads = process_thread_count();
    if (threads > threads_peak_) threads_peak_ = threads;
}

void PerformanceReport::add_collection(const CollectionCounters& counters,
                                       const StageTiming& timing) {
    collection_.add(counters);
    collection_timing_.ran = true;
    collection_timing_.wall_seconds += timing.wall_seconds;
    collection_timing_.cpu_user_seconds += timing.cpu_user_seconds;
    collection_timing_.cpu_system_seconds += timing.cpu_system_seconds;
    sample_threads();
}

void PerformanceReport::set_solve_timings(const RigTimings& timings) {
    timings_ = timings;
    sample_threads();
}

void PerformanceReport::set_dwecal(const DwecalTiming& dwecal) {
    dwecal_ = dwecal;
    sample_threads();
}

void PerformanceReport::finish(bool success, const std::string& message) {
    if (!started_ || finished_) return;
    finished_ = true;
    success_ = success;
    message_ = message;
    wall_seconds_ =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start_).count();
    process_cpu_end_ = process_cpu_times();
    peak_rss_kb_ = peak_rss_kb();
    sample_threads();
    finished_at_text_ = iso8601_now();
}

std::string PerformanceReport::to_yaml() const {
    std::ostringstream out;
    out << "run:\n";
    out << "  started_at: " << yaml_quote(started_at_text_) << "\n";
    out << "  finished_at: " << yaml_quote(finished_at_text_) << "\n";
    out << "  wall_seconds: " << number(wall_seconds_) << "\n";
    out << "  goal:\n";
    out << "    camera: " << yaml_quote(camera_) << "\n";
    out << "    budget: " << budget_ << "\n";
    out << "    attempts: " << attempts_ << "\n";
    out << "    success: " << (success_ ? "true" : "false") << "\n";
    out << "    message: " << yaml_quote(message_) << "\n";

    const double detected = static_cast<double>(collection_.frames_detected);
    const double collection_rate =
        collection_timing_.wall_seconds > 0.0 ? detected / collection_timing_.wall_seconds : 0.0;

    out << "collection:\n";
    out << "  frames_decoded: " << collection_.frames_decoded << "\n";
    out << "  frames_detected: " << collection_.frames_detected << "\n";
    out << "  frames_accepted: " << collection_.frames_accepted << "\n";
    out << "  frames_duplicate: " << collection_.frames_duplicate << "\n";
    out << "  frames_unpaired: " << collection_.frames_unpaired << "\n";
    out << "  detections_per_second: " << number(collection_rate) << "\n";
    out << "  wall_seconds: " << number(collection_timing_.wall_seconds) << "\n";
    out << "  cpu_user_seconds: " << number(collection_timing_.cpu_user_seconds) << "\n";
    out << "  cpu_system_seconds: " << number(collection_timing_.cpu_system_seconds) << "\n";

    out << "solve:\n";
    out << "  total_fit_seconds: " << number(timings_.mono_fit_seconds + timings_.joint_fit_seconds)
        << "\n";
    for (std::map<std::string, StageTiming>::const_iterator it = timings_.mono_solve.begin();
         it != timings_.mono_solve.end(); ++it) {
        write_stage(out, "  ", it->first, it->second);
    }
    write_stage(out, "  ", "joint_refine", timings_.joint_refine);
    out << "  joint_fit_seconds: " << number(timings_.joint_fit_seconds) << "\n";
    out << "  evaluate:\n";
    for (std::map<std::string, StageTiming>::const_iterator it = timings_.evaluate.begin();
         it != timings_.evaluate.end(); ++it) {
        write_stage(out, "    ", it->first, it->second);
    }
    write_stage(out, "  ", "artifacts", timings_.artifacts);

    out << "dwecal:\n";
    out << "  written: " << (dwecal_.timing.ran ? "true" : "false") << "\n";
    out << "  wall_seconds: " << number(dwecal_.timing.wall_seconds) << "\n";
    out << "  cpu_user_seconds: " << number(dwecal_.timing.cpu_user_seconds) << "\n";
    out << "  cpu_system_seconds: " << number(dwecal_.timing.cpu_system_seconds) << "\n";
    out << "  bytes: " << dwecal_.bytes << "\n";
    out << "  sha256: " << yaml_quote(dwecal_.sha256) << "\n";
    out << "  width: " << dwecal_.width << "\n";
    out << "  height: " << dwecal_.height << "\n";

    const double cpu_user = process_cpu_end_.user_seconds - process_cpu_start_.user_seconds;
    const double cpu_system = process_cpu_end_.system_seconds - process_cpu_start_.system_seconds;
    const double cpu_total = (cpu_user > 0.0 ? cpu_user : 0.0) + (cpu_system > 0.0 ? cpu_system : 0.0);
    const long cpus = sysconf(_SC_NPROCESSORS_ONLN);
    out << "cpu:\n";
    out << "  user_seconds: " << number(cpu_user > 0.0 ? cpu_user : 0.0) << "\n";
    out << "  system_seconds: " << number(cpu_system > 0.0 ? cpu_system : 0.0) << "\n";
    out << "  cpu_percent_of_one_core: "
        << number(wall_seconds_ > 0.0 ? 100.0 * cpu_total / wall_seconds_ : 0.0) << "\n";
    out << "  cpu_percent_of_all_cores: "
        << number(wall_seconds_ > 0.0 && cpus > 0
                      ? 100.0 * cpu_total / (wall_seconds_ * static_cast<double>(cpus))
                      : 0.0)
        << "\n";
    out << "  per_stage_worker_cpu:\n";
    out << "    collection: " << number(stage_cpu(collection_timing_)) << "\n";
    for (std::map<std::string, StageTiming>::const_iterator it = timings_.mono_solve.begin();
         it != timings_.mono_solve.end(); ++it) {
        out << "    mono_solve." << it->first << ": " << number(stage_cpu(it->second)) << "\n";
    }
    out << "    joint_refine: " << number(stage_cpu(timings_.joint_refine)) << "\n";
    for (std::map<std::string, StageTiming>::const_iterator it = timings_.evaluate.begin();
         it != timings_.evaluate.end(); ++it) {
        out << "    evaluate." << it->first << ": " << number(stage_cpu(it->second)) << "\n";
    }
    out << "    artifacts: " << number(stage_cpu(timings_.artifacts)) << "\n";
    out << "    dwecal: " << number(stage_cpu(timings_.dwecal)) << "\n";

    out << "memory:\n";
    out << "  peak_rss_kb: " << peak_rss_kb_ << "\n";
    out << "  threads_peak: " << threads_peak_ << "\n";

    struct utsname uts;
    std::memset(&uts, 0, sizeof(uts));
    const bool have_uts = uname(&uts) == 0;
    const char* ros_distro = std::getenv("ROS_DISTRO");
    out << "host:\n";
    out << "  cpus: " << (cpus > 0 ? cpus : 0) << "\n";
    out << "  kernel: " << yaml_quote(have_uts ? uts.release : "") << "\n";
    out << "  opencv_version: " << yaml_quote(CV_VERSION) << "\n";
    out << "  ceres_version: " << yaml_quote(CERES_VERSION_STRING) << "\n";
    out << "  ros_distro: " << yaml_quote(ros_distro != NULL ? ros_distro : "") << "\n";

    if (warnings_.empty()) {
        out << "warnings: []\n";
    } else {
        out << "warnings:\n";
        for (std::size_t i = 0; i < warnings_.size(); ++i) {
            out << "  - " << yaml_quote(warnings_[i]) << "\n";
        }
    }
    return out.str();
}

bool PerformanceReport::write(const std::string& path, std::string* error) const {
    return write_text_file_atomic(path, to_yaml(), error);
}

}  // namespace discocal_ros
