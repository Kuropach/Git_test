#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

// Простая функция для чтения JSON значения
float read_json_value(const std::string& json_str, const std::string& key) {
    size_t pos = json_str.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0.0f;
    
    pos = json_str.find(":", pos);
    if (pos == std::string::npos) return 0.0f;
    
    size_t end = json_str.find(",", pos);
    if (end == std::string::npos) end = json_str.find("}", pos);
    
    std::string value_str = json_str.substr(pos + 1, end - pos - 1);
    
    // Убираем пробелы и кавычки
    value_str.erase(0, value_str.find_first_not_of(" \t\n\r\""));
    value_str.erase(value_str.find_last_not_of(" \t\n\r\"") + 1);
    
    return std::stof(value_str);
}

// Функция для чтения строкового значения из JSON
std::string read_json_string(const std::string& json_str, const std::string& key) {
    size_t pos = json_str.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    
    pos = json_str.find(":", pos);
    if (pos == std::string::npos) return "";
    
    size_t start_quote = json_str.find("\"", pos);
    if (start_quote == std::string::npos) return "";
    
    size_t end_quote = json_str.find("\"", start_quote + 1);
    if (end_quote == std::string::npos) return "";
    
    return json_str.substr(start_quote + 1, end_quote - start_quote - 1);
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file.json>" << std::endl;
        return 1;
    }

    // Чтение всего файла конфигурации
    std::ifstream config_file(argv[1]);
    if (!config_file.is_open()) {
        std::cerr << "Cannot open config file: " << argv[1] << std::endl;
        return 1;
    }

    std::stringstream buffer;
    buffer << config_file.rdbuf();
    std::string json_str = buffer.str();
    config_file.close();

    // Извлечение параметров
    float x0 = read_json_value(json_str, "initial_position");
    float v0 = read_json_value(json_str, "initial_velocity");
    float dt = read_json_value(json_str, "dt");
    float w = read_json_value(json_str, "w");
    float sim_time = read_json_value(json_str, "simulation_time");
    std::string output_file = read_json_string(json_str, "output_file");
    std::string method = read_json_string(json_str, "method");

    // Проверка значений по умолчанию
    if (output_file.empty()) output_file = "results.csv";
    if (method.empty()) method = "RK4";

    std::vector<float> x = {x0};
    std::vector<float> v = {v0};
    float t = 0;
    int i = 0;

    // Открытие файла для результатов
    std::ofstream out(output_file);
    out << "time,position,velocity,energy" << std::endl;
    out << t << "," << x[i] << "," << v[i] << "," << (v[i]*v[i]/2 + w*w*x[i]*x[i]/2) << std::endl;

    // Основной цикл симуляции
    while (t < sim_time) {
        if (method == "RK4") {
            // Метод Рунге-Кутта 4-го порядка
            float kx1 = v[i];
            float kv1 = -w*w*x[i];
            
            float kx2 = v[i] + dt/2 * kv1;
            float kv2 = -w*w*(x[i] + dt/2 * kx1);
            
            float kx3 = v[i] + dt/2 * kv2;
            float kv3 = -w*w*(x[i] + dt/2 * kx2);
            
            float kx4 = v[i] + dt * kv3;
            float kv4 = -w*w*(x[i] + dt * kx3);
            
            x.push_back(x[i] + dt/6 * (kx1 + 2*kx2 + 2*kx3 + kx4));
            v.push_back(v[i] + dt/6 * (kv1 + 2*kv2 + 2*kv3 + kv4));
        } else {
            // Улучшенный метод Эйлера
            float x_pred = x[i] + dt * v[i];
            float v_pred = v[i] + dt * (-w * w * x[i]);
            
            x.push_back(x[i] + dt * 0.5f * (v[i] + v_pred));
            v.push_back(v[i] + dt * 0.5f * (-w * w * (x[i] + x_pred)));
        }

        i++;
        t = dt * i;

        // Запись результатов
        out << t << "," << x[i] << "," << v[i] << "," << (v[i]*v[i]/2 + w*w*x[i]*x[i]/2) << std::endl;
    }

    out.close();
    std::cout << "Simulation completed. Results saved to: " << output_file << std::endl;
    return 0;
}