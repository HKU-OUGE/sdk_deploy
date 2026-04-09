#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include "json.hpp" // 依赖 nlohmann json
#include "m20_sensor_policy_runner.hpp"

using json = nlohmann::json;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_onnx> <path_to_logbag.jsonl>\n";
        return 1;
    }
    
    std::string onnx_path = argv[1];
    std::string log_path = argv[2];

    // 初始化 Runner，第三个参数为 true 表示开启离线测试，屏蔽 ROS 初始化
    M20SensorPolicyRunner runner("offline_test", onnx_path, true);
    RobotBasicState dummy_rbs;
    runner.OnEnter(dummy_rbs); // 初始化 h0 和历史缓存

    std::ifstream infile(log_path);
    if (!infile.is_open()) {
        std::cerr << "Failed to open logbag: " << log_path << "\n";
        return 1;
    }

    std::string line;
    int step = 0;
    float max_mse = 0;

    while (std::getline(infile, line)) {
        if (line.empty()) continue;
        step++;
        
        json data = json::parse(line);

        std::vector<float> omega = data["omega"];
        std::vector<float> proj_g = data["proj_g"];
        std::vector<float> cmd = data["cmd"];
        std::vector<float> jp = data["jp"];
        std::vector<float> jv = data["jv"];
        std::vector<float> last_action = data["last_action"];
        std::vector<float> heights = data["processed_heights"];
        std::vector<float> fwd_scan = data["raw_fwd"];
        std::vector<float> bwd_scan = data["raw_bwd"];
        std::vector<float> gt_action = data["gt_action"];

        // 离线执行处理与推理
        VecXf cxx_action = runner.testOfflineStep(
            omega, proj_g, cmd, jp, jv, last_action, heights, fwd_scan, bwd_scan);

        // 对比结果计算 MSE
        float mse = 0;
        for (int i = 0; i < 16; ++i) {
            float diff = cxx_action(i) - gt_action[i];
            mse += diff * diff;
        }
        mse /= 16.0f;

        if (mse > max_mse) max_mse = mse;

        if (mse > 1e-4) {
            std::cout << "[Step " << step << "] WARNING: MSE is high (" << mse << ")\n";
            std::cout << "  C++ Output: " << cxx_action.transpose() << "\n";
            std::cout << "  Py  Output: ";
            for (float a : gt_action) std::cout << a << " ";
            std::cout << "\n";
        }
    }

    std::cout << "\n[Done] Tested " << step << " steps.\n";
    std::cout << "Max MSE across all steps: " << max_mse << "\n";
    
    if (max_mse < 1e-4) {
        std::cout << "\033[92mSUCCESS: C++ implementation perfectly matches Python!\033[0m\n";
    } else {
        std::cout << "\033[91mFAILED: Significant discrepancy detected. Check mappings and scalings.\033[0m\n";
    }

    return 0;
}