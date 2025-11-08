/**
 *
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 *
 */

#include <interrupts_101310113_101308951.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>

static int small_step_ms(int seed_bump = 0) {
    static int seed = 7;
    seed = (seed * 1103515245 + 12345 + seed_bump) & 0x7fffffff;
    return 1 + (seed % 10);
}

// Next PID for forked children (PID 0 is initial)
static unsigned int g_next_pid = 1;

// Directory of top-level trace file (for programX.txt)
static std::string g_trace_dir;

static void append_system_status(std::string &system_status,
                                 int current_time,
                                 const std::string &trace_line,
                                 PCB current,
                                 const std::vector<PCB> &wait_queue)
{
    system_status += "time: " + std::to_string(current_time)
                  +  "; current trace: " + trace_line + "\n";
    system_status += print_PCB(current, wait_queue);
    system_status += "\n";
}

std::tuple<std::string, std::string, int>
simulate_trace(const std::vector<std::string> &trace_file,
               int time,
               const std::vector<std::string> &vectors,
               const std::vector<int> &delays,
               const std::vector<external_file> &external_files,
               PCB current,
               std::vector<PCB> wait_queue);

std::tuple<std::string, std::string, int>
simulate_trace(const std::vector<std::string> &trace_file,
               int time,
               const std::vector<std::string> &vectors,
               const std::vector<int> &delays,
               const std::vector<external_file> &external_files,
               PCB current,
               std::vector<PCB> wait_queue)
{
    std::string execution;
    std::string system_status;
    int current_time = time;

    for (std::size_t i = 0; i < trace_file.size(); ++i) {

        std::string trace = trace_file[i];
        auto [activity, duration_intr, program_name] = parse_trace(trace);

        if (activity == "CPU") {

            execution += std::to_string(current_time) + ", "
                       + std::to_string(duration_intr)
                       + ", CPU Burst\n";
            current_time += duration_intr;
        }
        else if (activity == "SYSCALL") {

            auto [intr, t2] = intr_boilerplate(current_time,
                                               duration_intr,
                                               10,
                                               vectors);
            execution += intr;
            current_time = t2;

            execution += std::to_string(current_time) + ", "
                       + std::to_string(delays[duration_intr])
                       + ", SYSCALL ISR\n";
            current_time += delays[duration_intr];

            execution += std::to_string(current_time) + ", 1, IRET\n";
            current_time += 1;
        }
        else if (activity == "END_IO" || activity == "ENDIO") {

            auto [intr, t2] = intr_boilerplate(current_time,
                                               duration_intr,
                                               10,
                                               vectors);
            execution += intr;
            current_time = t2;

            execution += std::to_string(current_time) + ", "
                       + std::to_string(delays[duration_intr])
                       + ", ENDIO ISR\n";
            current_time += delays[duration_intr];

            execution += std::to_string(current_time) + ", 1, IRET\n";
            current_time += 1;
        }
        else if (activity == "FORK") {

            auto [intr, t2] = intr_boilerplate(current_time,
                                               2,
                                               10,
                                               vectors);
            execution += intr;
            current_time = t2;

            execution += std::to_string(current_time) + ", "
                       + std::to_string(duration_intr)
                       + ", cloning the PCB\n";
            current_time += duration_intr;

            PCB child(g_next_pid++,
                      static_cast<int>(current.PID),
                      current.program_name,
                      current.size,
                      -1);

            bool allocated = allocate_memory(&child);

            std::vector<PCB> fork_wait_queue = wait_queue;
            fork_wait_queue.push_back(current);

            execution += std::to_string(current_time) + ", 0, scheduler called\n";
            execution += std::to_string(current_time) + ", 1, IRET\n";
            current_time += 1;

            if (!allocated) {
                append_system_status(system_status, current_time, trace, current, wait_queue);
                continue;
            }

            append_system_status(system_status, current_time, trace, child, fork_wait_queue);

            std::vector<std::string> child_trace;
            bool skip = true;
            bool exec_seen = false;
            std::size_t parent_index = i;

            for (std::size_t j = i + 1; j < trace_file.size(); ++j) {
                auto [act_j, dur_j, prog_j] = parse_trace(trace_file[j]);

                if (skip && act_j == "IF_CHILD") {
                    skip = false;
                    continue;
                }
                else if (act_j == "IF_PARENT") {
                    skip = true;
                    parent_index = j;
                    if (exec_seen) break;
                    continue;
                }
                else if (skip && act_j == "ENDIF") {
                    skip = false;
                    continue;
                }
                else if (!skip && act_j == "EXEC") {
                    child_trace.push_back(trace_file[j]);
                    exec_seen = true;
                    skip = true;
                    continue;
                }

                if (!skip) {
                    child_trace.push_back(trace_file[j]);
                }
            }

            i = parent_index;

            auto [e_child, s_child, t_child] =
                simulate_trace(child_trace,
                               current_time,
                               vectors,
                               delays,
                               external_files,
                               child,
                               fork_wait_queue);

            execution     += e_child;
            system_status += s_child;
            current_time   = t_child;

            free_memory(&child);
        }
        else if (activity == "EXEC") {

            auto [intr, t2] = intr_boilerplate(current_time,
                                               3,
                                               10,
                                               vectors);
            execution += intr;
            current_time = t2;

            unsigned int prog_size_mb = get_size(program_name, external_files);

            execution += std::to_string(current_time) + ", "
                       + std::to_string(duration_intr)
                       + ", Program is "
                       + std::to_string(prog_size_mb)
                       + " Mb large\n";
            current_time += duration_intr;

            int load_ms = static_cast<int>(prog_size_mb) * 15;
            execution += std::to_string(current_time) + ", "
                       + std::to_string(load_ms)
                       + ", loading program into memory\n";
            current_time += load_ms;

            free_memory(&current);
            current.program_name = program_name;
            current.size         = prog_size_mb;

            bool allocated = allocate_memory(&current);

            int mark_ms = small_step_ms(1);
            execution += std::to_string(current_time) + ", "
                       + std::to_string(mark_ms)
                       + ", marking partition as occupied\n";
            current_time += mark_ms;

            int upd_ms = small_step_ms(2);
            execution += std::to_string(current_time) + ", "
                       + std::to_string(upd_ms)
                       + ", updating PCB\n";
            current_time += upd_ms;

            execution += std::to_string(current_time) + ", 0, scheduler called\n";
            execution += std::to_string(current_time) + ", 1, IRET\n";
            current_time += 1;

            append_system_status(system_status, current_time, trace, current, wait_queue);

            std::vector<std::string> exec_traces;
            std::string exec_line;

            std::ifstream exec_trace_file(g_trace_dir + program_name + ".txt");
            if (exec_trace_file.is_open()) {
                while (std::getline(exec_trace_file, exec_line)) {
                    if (!exec_line.empty()) {
                        exec_traces.push_back(exec_line);
                    }
                }
                exec_trace_file.close();
            }

            if (allocated && !exec_traces.empty()) {
                auto [e_exec, s_exec, t_exec] =
                    simulate_trace(exec_traces,
                                   current_time,
                                   vectors,
                                   delays,
                                   external_files,
                                   current,
                                   wait_queue);

                execution     += e_exec;
                system_status += s_exec;
                current_time   = t_exec;
            }

            break;
        }
        // IF_CHILD / IF_PARENT / ENDIF handled via FORK parsing, ignored here.
    }

    return std::make_tuple(execution, system_status, current_time);
}

int main(int argc, char** argv)
{
    auto [vectors, delays, external_files] = parse_args(argc, argv);

    print_external_files(external_files);

    std::string top_trace_path = argv[1];
    std::size_t pos = top_trace_path.find_last_of("/\\");
    if (pos == std::string::npos) {
        g_trace_dir.clear();
    } else {
        g_trace_dir = top_trace_path.substr(0, pos + 1);
    }

    PCB current(0, -1, "init", 1, -1);

    if (!allocate_memory(&current)) {
        std::cerr << "ERROR! Memory allocation for init failed!" << std::endl;
    }

    std::vector<PCB> wait_queue;

    std::ifstream input_file(top_trace_path);
    if (!input_file.is_open()) {
        std::cerr << "ERROR! Could not open top-level trace file: "
                  << top_trace_path << std::endl;
        return 1;
    }

    std::vector<std::string> trace_file;
    std::string trace_line;

    while (std::getline(input_file, trace_line)) {
        if (!trace_line.empty()) {
            trace_file.push_back(trace_line);
        }
    }
    input_file.close();

    auto [execution, system_status, end_time] =
        simulate_trace(trace_file,
                       0,
                       vectors,
                       delays,
                       external_files,
                       current,
                       wait_queue);

    write_output(execution, "execution.txt");
    write_output(system_status, "system_status.txt");

    return 0;
}
