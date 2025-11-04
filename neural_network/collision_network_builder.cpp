#include <torch/torch.h>
#include <iostream>
#include <tuple>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <filesystem>

class CustomDataset : public torch::data::Dataset<CustomDataset>{
    public:
        explicit CustomDataset(const std::string& filename, const int& num_inputs, const int& num_outputs) {
            std::vector<double> features;
            std::vector<double> labels;
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::string line;
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string cell;
                int i = 0;
                while (std::getline(ss, cell, ',')) {
                    if (i++ < num_inputs) {
                        features.push_back(std::stod(cell));
                    } else {
                        labels.push_back(std::stod(cell));
                    }
                }
            }
            file.close();
            long long num_samples = features.size() / num_inputs;
            features_tensor_ = torch::from_blob(features.data(), {num_samples, num_inputs}, torch::TensorOptions().dtype(torch::kDouble)).clone().to(torch::kFloat);
            labels_tensor_ = torch::from_blob(labels.data(), {num_samples, num_outputs}, torch::TensorOptions().dtype(torch::kDouble)).clone().to(torch::kFloat);
        }
        torch::data::Example<> get(size_t index) override {
            long long long_index = static_cast<long long>(index);
            return {features_tensor_.index({long_index}), labels_tensor_.index({long_index})};
        }
        torch::optional<size_t> size() const override {
            return features_tensor_.size(0);
        }
    private:
        torch::Tensor features_tensor_;
        torch::Tensor labels_tensor_;
};


struct Net : torch::nn::Module {
  Net(const int& num_inputs, const int& num_outputs) {
    int middle = (num_inputs+num_outputs)/2;
    fc1 = register_module("fc1", torch::nn::Linear(num_inputs, middle));
    fc2 = register_module("fc2", torch::nn::Linear(middle, num_outputs));
  }
  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = fc2->forward(x);
    return x;
  }
  torch::nn::Linear fc1{nullptr}, fc2{nullptr};
};

int main() {
    std::string training_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/collision_data/raw/training.csv";
    std::string testing_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/collision_data/raw/testing.csv";
    std::string model_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/networks/raw/net5.pt";
    std::string output_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/output_data/raw/output5.csv";

    int num_inputs = 22;
    int num_outputs = 3;

    auto base_dataset = CustomDataset(training_path, num_inputs, num_outputs);
    const size_t dataset_size = base_dataset.size().value();
    auto mapped_dataset = std::move(base_dataset).map(torch::data::transforms::Stack<>());
    auto data_loader = torch::data::make_data_loader(
        std::move(mapped_dataset),
        torch::data::samplers::RandomSampler(dataset_size),
        torch::data::DataLoaderOptions().batch_size(32)
    );
    auto net = std::make_shared<Net>(num_inputs, num_outputs);
    torch::nn::MSELoss criterion;
    torch::optim::Adam optimizer(net->parameters(), 0.001);
    for (size_t epoch = 1; epoch <= 5; ++epoch) {
        size_t batch_index = 0;
        for (auto& batch : *data_loader) {
            optimizer.zero_grad();
            torch::Tensor output = net->forward(batch.data);
            torch::Tensor loss = criterion(output, batch.target);
            loss.backward();
            optimizer.step();
        }
    }
    torch::save(net, model_path);
    auto base_test_dataset = CustomDataset(testing_path, num_inputs, num_outputs);
    const size_t test_dataset_size = base_test_dataset.size().value();
    auto mapped_test_dataset = std::move(base_test_dataset).map(torch::data::transforms::Stack<>());
    auto test_data_loader = torch::data::make_data_loader(
        std::move(mapped_test_dataset),
        torch::data::samplers::SequentialSampler(test_dataset_size),
        torch::data::DataLoaderOptions().batch_size(32)
    );
    for (auto& batch : *test_data_loader) {
        torch::Tensor test_output = net->forward(batch.data);
        std::ofstream file(output_path, std::ios::app);
        for (size_t i = 0; i < test_output.size(0); i++) {
            for (size_t j = 0; j < test_output.size(1); j++) {
                file << test_output[i][j].item() << ",";
            }
            file << std::endl;
        }
    }
    return 0;
}