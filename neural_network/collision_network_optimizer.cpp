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

struct Net1 : torch::nn::Module {
  Net1(const int& num_inputs, const int& num_outputs, const double& dropout_rate) {
    int middle = (num_inputs+num_outputs)/2;
    fc1 = register_module("fc1", torch::nn::Linear(num_inputs, middle));
    dropout = register_module("dropout", torch::nn::Dropout(dropout_rate));
    fc2 = register_module("fc2", torch::nn::Linear(middle, num_outputs));
  }
  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = dropout->forward(x);
    x = fc2->forward(x);
    return x;
  }
  torch::nn::Linear fc1{nullptr}, fc2{nullptr};
  torch::nn::Dropout dropout{nullptr};
};

struct Net2 : torch::nn::Module {
  Net2(const int& num_inputs, const int& num_outputs, const double& dropout_rate) {
    fc1 = register_module("fc1", torch::nn::Linear(num_inputs, 32));
    dropout1 = register_module("dropout1", torch::nn::Dropout(dropout_rate));
    fc2 = register_module("fc2", torch::nn::Linear(32, 64));
    dropout2 = register_module("dropout2", torch::nn::Dropout(dropout_rate));
    fc3 = register_module("fc3", torch::nn::Linear(64, num_outputs));
  }
  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = dropout1->forward(x);
    x = torch::relu(fc2->forward(x));
    x = dropout2->forward(x);
    x = fc3->forward(x);
    return x;
  }
  torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
  torch::nn::Dropout dropout1{nullptr}, dropout2{nullptr};
};

struct Net3 : torch::nn::Module {
  Net3(const int& num_inputs, const int& num_outputs, const double& dropout_rate) {
    fc1 = register_module("fc1", torch::nn::Linear(num_inputs, 32));
    dropout1 = register_module("dropout1", torch::nn::Dropout(dropout_rate));
    fc2 = register_module("fc2", torch::nn::Linear(32, 64));
    dropout2 = register_module("dropout2", torch::nn::Dropout(dropout_rate));
    fc3 = register_module("fc3", torch::nn::Linear(64, 32));
    dropout3 = register_module("dropout3", torch::nn::Dropout(dropout_rate));
    fc4 = register_module("fc4", torch::nn::Linear(32, num_outputs));
  }
  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = dropout1->forward(x);
    x = torch::relu(fc2->forward(x));
    x = dropout2->forward(x);
    x = torch::relu(fc3->forward(x));
    x = dropout3->forward(x);
    x = fc4->forward(x);
    return x;
  }
  torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr}, fc4{nullptr};
  torch::nn::Dropout dropout1{nullptr}, dropout2{nullptr}, dropout3{nullptr};
};

struct Net4 : torch::nn::Module {
  Net4(const int& num_inputs, const int& num_outputs, const double& dropout_rate) {
    fc1 = register_module("fc1", torch::nn::Linear(num_inputs, 25));
    dropout1 = register_module("dropout1", torch::nn::Dropout(dropout_rate));
    fc2 = register_module("fc2", torch::nn::Linear(25, 25));
    dropout2 = register_module("dropout2", torch::nn::Dropout(dropout_rate));
    fc3 = register_module("fc3", torch::nn::Linear(25, 15));
    dropout3 = register_module("dropout3", torch::nn::Dropout(dropout_rate));
    fc4 = register_module("fc4", torch::nn::Linear(15, num_outputs));
  }
  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = dropout1->forward(x);
    x = torch::relu(fc2->forward(x));
    x = dropout2->forward(x);
    x = torch::relu(fc3->forward(x));
    x = dropout3->forward(x);
    x = fc4->forward(x);
    return x;
  }
  torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr}, fc4{nullptr};
  torch::nn::Dropout dropout1{nullptr}, dropout2{nullptr}, dropout3{nullptr};
};

int main() {
    std::string training_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/collision_data/z/training.csv";
    std::string validation_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/collision_data/z/validation.csv";
    std::string testing_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/collision_data/z/testing.csv";
    std::string model_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/networks/temp/net.pt";
    int num_inputs = 22;
    int num_outputs = 3;

    auto base_validation_dataset = CustomDataset(validation_path, num_inputs, num_outputs);
    const size_t validation_dataset_size = base_validation_dataset.size().value();
    auto mapped_validation_dataset = std::move(base_validation_dataset).map(torch::data::transforms::Stack<>());
    auto validation_data_loader = torch::data::make_data_loader(
        std::move(mapped_validation_dataset),
        torch::data::samplers::SequentialSampler(validation_dataset_size),
        torch::data::DataLoaderOptions().batch_size(validation_dataset_size)
    );
    auto base_test_dataset = CustomDataset(testing_path, num_inputs, num_outputs);
    const size_t test_dataset_size = base_test_dataset.size().value();
    auto mapped_test_dataset = std::move(base_test_dataset).map(torch::data::transforms::Stack<>());
    auto test_data_loader = torch::data::make_data_loader(
        std::move(mapped_test_dataset),
        torch::data::samplers::SequentialSampler(test_dataset_size),
        torch::data::DataLoaderOptions().batch_size(test_dataset_size)
    );
    std::ofstream file("/Users/jacobcollier-tenison/GitHub/capstone_project/output_data/loss_comparisons/loss.csv", std::ios::app);
    std::vector<double> learning_rates = {.0001, .0005, .001, .005, .01, .05};
    std::vector<int> batch_sizes = {1, 2, 4, 8, 16, 32, 64, 128};
    std::vector<double> dropouts = {0, .1, .2, .3, .4, .5, .6};
    std::vector<int> model_architectures = {1, 2, 3, 4};
    
    for (const auto& batch_size_param : batch_sizes) {
        auto base_dataset = CustomDataset(training_path, num_inputs, num_outputs);
        const size_t dataset_size = base_dataset.size().value();
        auto mapped_dataset = std::move(base_dataset).map(torch::data::transforms::Stack<>());
        auto data_loader = torch::data::make_data_loader(
            std::move(mapped_dataset),
            torch::data::samplers::RandomSampler(dataset_size),
            torch::data::DataLoaderOptions().batch_size(batch_size_param)
        );
        for (const auto& learning_rate : learning_rates) {
            for (const auto& dropout : dropouts) {
                for (const auto& model_architecture : model_architectures) {
                    std::shared_ptr<torch::nn::Module> net;
                    std::shared_ptr<torch::nn::Module> eval_net;
                    if (model_architecture == 1){
                        net = std::make_shared<Net1>(num_inputs, num_outputs, dropout);
                        eval_net = std::make_shared<Net1>(num_inputs, num_outputs, dropout);
                    } else if (model_architecture == 2) {
                        net = std::make_shared<Net2>(num_inputs, num_outputs, dropout);
                        eval_net = std::make_shared<Net2>(num_inputs, num_outputs, dropout);
                    } else if (model_architecture == 3) {
                        net = std::make_shared<Net3>(num_inputs, num_outputs, dropout);
                        eval_net = std::make_shared<Net3>(num_inputs, num_outputs, dropout);
                    } else if (model_architecture == 4) {
                        net = std::make_shared<Net4>(num_inputs, num_outputs, dropout);
                        eval_net = std::make_shared<Net4>(num_inputs, num_outputs, dropout);
                    }
                    torch::nn::MSELoss criterion;
                    torch::optim::Adam optimizer(net->parameters(), learning_rate);
                    double least_loss = 0;
                    int epochs_since_least = 0;
                    double validation_loss = 0;
                    int epochs;
                    for (size_t epoch = 0; epoch <= 100000; ++epoch) {
                        for (auto& batch : *data_loader) {
                            optimizer.zero_grad();
                            torch::Tensor output;
                            if (model_architecture == 1) {
                                output = std::static_pointer_cast<Net1>(net)->forward(batch.data);
                            } else if (model_architecture == 2) {
                                output = std::static_pointer_cast<Net2>(net)->forward(batch.data);
                            } else if (model_architecture == 3) {
                                output = std::static_pointer_cast<Net3>(net)->forward(batch.data);
                            } else if (model_architecture == 4) {
                                output = std::static_pointer_cast<Net4>(net)->forward(batch.data);
                            }
                            torch::Tensor loss = criterion(output, batch.target);
                            loss.backward();
                            optimizer.step();
                        }
                        net->eval();
                        torch::NoGradGuard no_grad; 
                        for (auto& batch : *validation_data_loader) {
                            torch::Tensor output;
                            if (model_architecture == 1) {
                                output = std::static_pointer_cast<Net1>(net)->forward(batch.data);
                            } else if (model_architecture == 2) {
                                output = std::static_pointer_cast<Net2>(net)->forward(batch.data);
                            } else if (model_architecture == 3) {
                                output = std::static_pointer_cast<Net3>(net)->forward(batch.data);
                            } else if (model_architecture == 4) {
                                output = std::static_pointer_cast<Net4>(net)->forward(batch.data);
                            }
                            validation_loss = criterion(output, batch.target).item<double>();
                        }
                        net->train();
                        if ((validation_loss < least_loss) || (epoch == 0)) {
                            torch::save(net, model_path);
                            least_loss = validation_loss;
                            epochs_since_least = 0;
                        }
                        else if (++epochs_since_least == 100) {
                            epochs = epoch - 100;
                            break;
                        }
                    }
                    torch::load(eval_net, model_path);
                    eval_net->eval();
                    torch::NoGradGuard no_grad;
                    double testing_loss;
                    for (auto& batch : *test_data_loader) {
                        torch::Tensor test_output;
                        if (model_architecture == 1) {
                            test_output = std::static_pointer_cast<Net1>(eval_net)->forward(batch.data);
                        } else if (model_architecture == 2) {
                            test_output = std::static_pointer_cast<Net2>(eval_net)->forward(batch.data);
                        } else if (model_architecture == 3) {
                            test_output = std::static_pointer_cast<Net3>(eval_net)->forward(batch.data);
                        } else if (model_architecture == 4) {
                            test_output = std::static_pointer_cast<Net4>(eval_net)->forward(batch.data);
                        }
                        testing_loss = criterion(test_output, batch.target).item<double>();
                    }
                    file << batch_size_param << ", " << learning_rate << ", " << dropout << ", " 
                        << model_architecture << ", " << epochs << ", " << testing_loss << std::endl;
                }                
            }
        }
    }
    return 0;
}