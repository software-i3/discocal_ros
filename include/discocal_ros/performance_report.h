#ifndef DISCOCAL_ROS_PERFORMANCE_REPORT_H_
#define DISCOCAL_ROS_PERFORMANCE_REPORT_H_

// Timing and resource accounting for one calibration goal, written next to the
// calibration artifacts as performance.yaml.
//
// Two different clocks are recorded for every stage:
//
//   * wall time, from std::chrono::steady_clock, which is what a user waits
//     for; and
//   * CPU time of the calling thread, from getrusage(RUSAGE_THREAD), which is
//     what the worker actually spent. A stage that waits on images has wall
//     time but little CPU; a Ceres stage has both.
//
// Stage timings are measured where the work happens (RigSession for the solve
// stages, the node for collection), not reconstructed from log timestamps.
// GPU work -- there is none here -- would not appear in either number.

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace discocal_ros {

struct StageTiming {
    bool ran = false;
    double wall_seconds = 0.0;
    double cpu_user_seconds = 0.0;
    double cpu_system_seconds = 0.0;
};

struct CpuTimes {
    double user_seconds = 0.0;
    double system_seconds = 0.0;
};

// CPU consumed by the whole process (all threads).
CpuTimes process_cpu_times();
// CPU consumed by the calling thread only. On Linux this is the worker thread;
// it is the number that attributes cost to a stage rather than to the spinners.
CpuTimes thread_cpu_times();
// Threads in this process right now, from /proc/self/status.
int process_thread_count();

// Wall and calling-thread CPU since construction; stop() reports the delta.
// stop() may be called once; a second call would report the deltas since the
// first, which is almost never what a caller means.
class StageClock {
public:
    StageClock();
    StageTiming stop();

private:
    std::chrono::steady_clock::time_point wall_start_;
    CpuTimes cpu_start_;
    bool stopped_ = false;
};

// Frames are counted per attempt and summed by PerformanceReport::add_collection,
// so a retried goal reports the work it actually did, not just the last try.
struct CollectionCounters {
    long long frames_decoded = 0;
    long long frames_detected = 0;
    long long frames_accepted = 0;
    long long frames_duplicate = 0;
    long long frames_unpaired = 0;

    void add(const CollectionCounters& other);
};

// Filled by RigSession::solve(). Maps are keyed by camera name.
struct RigTimings {
    std::map<std::string, StageTiming> mono_solve;
    double mono_fit_seconds = 0.0;  // sum of the library's reported fit times
    StageTiming joint_refine;
    double joint_fit_seconds = 0.0;
    std::map<std::string, StageTiming> evaluate;
    // The whole artifact phase, including the .dwecal.
    StageTiming artifacts;
    // The .dwecal generation and write, a subset of `artifacts`.
    StageTiming dwecal;
};

struct DwecalTiming {
    int width = 0;
    int height = 0;
    std::uint64_t bytes = 0;
    std::string sha256;
    StageTiming timing;
};

class PerformanceReport {
public:
    PerformanceReport();

    void begin(const std::string& camera, int budget);
    bool started() const { return started_; }

    void add_collection(const CollectionCounters& counters, const StageTiming& timing);
    void set_solve_timings(const RigTimings& timings);
    void set_dwecal(const DwecalTiming& dwecal);
    void set_attempts(int attempts) { attempts_ = attempts; }
    void add_warning(const std::string& warning) { warnings_.push_back(warning); }

    // Captures the process totals and the end time. Call once, after the goal
    // has finished (successfully or not).
    void finish(bool success, const std::string& message);

    std::string to_yaml() const;
    // Atomic write; see camera_info_writer.h.
    bool write(const std::string& path, std::string* error) const;

private:
    void sample_threads();

    bool started_ = false;
    std::chrono::steady_clock::time_point wall_start_;
    CpuTimes process_cpu_start_;
    std::string started_at_text_;
    std::string finished_at_text_;
    bool finished_ = false;
    double wall_seconds_ = 0.0;

    std::string camera_;
    int budget_ = 0;
    int attempts_ = 0;
    bool success_ = false;
    std::string message_;

    CollectionCounters collection_;
    StageTiming collection_timing_;
    RigTimings timings_;
    DwecalTiming dwecal_;

    CpuTimes process_cpu_end_;
    long peak_rss_kb_ = 0;
    int threads_peak_ = 0;
    std::vector<std::string> warnings_;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_PERFORMANCE_REPORT_H_
