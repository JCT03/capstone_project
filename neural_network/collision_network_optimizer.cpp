#include <torch/torch.h>
#include <iostream>
#include <tuple>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <filesystem>

// Custom dataset class to allow data to be read into a Torch dataset
class CustomDataset : public torch::data::Dataset<CustomDataset>{
    public:
        explicit CustomDataset(const std::string& filename, const int& num_inputs,
            const int& num_outputs) {
            // Creates vectors to store features and labels
            std::vector<double> features;
            std::vector<double> labels;

            // Opens input stream to file
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                std::exit(EXIT_FAILURE);
            }

            // Declares variable for file line
            std::string line;

            // Reads each line from file
            while (std::getline(file, line)) {
                // Creates stream to line
                std::stringstream ss(line);

                // Declares variable to save each file
                std::string cell;
                int i = 0;
                // Pushes first n values onto features vector and rest onto labels
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

            // Saves features as tensor
            features_tensor_ = torch::from_blob(features.data(),
                {num_samples, num_inputs}, torch::TensorOptions().dtype(torch::kDouble))
                .clone().to(torch::kFloat);

            // Saves labels as tensor
            labels_tensor_ = torch::from_blob(labels.data(), {num_samples, num_outputs},
                torch::TensorOptions().dtype(torch::kDouble)).clone().to(torch::kFloat);
        }
        // Function to return features and labels at given index
        torch::data::Example<> get(size_t index) override {
            long long long_index = static_cast<long long>(index);
            return {features_tensor_.index({long_index}), 
                labels_tensor_.index({long_index})};
        }

        // Function to return number of rows in dataset
        torch::optional<size_t> size() const override {
            return features_tensor_.size(0);
        }
    private:
        // Declares features and label tensors
        torch::Tensor features_tensor_;
        torch::Tensor labels_tensor_;
};

// Defines first neural network architecture
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

// Defines second neural network architecture
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

// Defines third neural network architecture
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

// Defines fourth neural network architecture
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
    // Declares and intializes paths to data and to save networks temporarily
    std::string training_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/
        collision_data/z/training.csv";
    std::string validation_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/
        collision_data/z/validation.csv";
    std::string testing_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/
        collision_data/z/testing.csv";
    std::string model_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/
        networks/temp/net.pt";

    // Declares and initializes number of inputs and outputs to model
    int num_inputs = 22;
    int num_outputs = 3;

    // Creates dataset and dataloader with validation set
    auto base_validation_dataset = CustomDataset(validation_path, num_inputs, 
        num_outputs);
    const size_t validation_dataset_size = base_validation_dataset.size().value();
    auto mapped_validation_dataset = std::move(base_validation_dataset).map(torch::data::
        transforms::Stack<>());
    auto validation_data_loader = torch::data::make_data_loader(
        std::move(mapped_validation_dataset),
        torch::data::samplers::SequentialSampler(validation_dataset_size),
        torch::data::DataLoaderOptions().batch_size(validation_dataset_size)
    );

    // Creates dataset and dataloader with testing set
    auto base_test_dataset = CustomDataset(testing_path, num_inputs, num_outputs);
    const size_t test_dataset_size = base_test_dataset.size().value();
    auto mapped_test_dataset = std::move(base_test_dataset).map(torch::data::transforms::
        Stack<>());
    auto test_data_loader = torch::data::make_data_loader(
        std::move(mapped_test_dataset),
        torch::data::samplers::SequentialSampler(test_dataset_size),
        torch::data::DataLoaderOptions().batch_size(test_dataset_size)
    );

    // Opens stream to write loss values to csv file
    std::ofstream file("/Users/jacobcollier-tenison/GitHub/capstone_project/output_data/
        loss_comparisons/sgdm_loss.csv", std::ios::app);
    
    // Creates vectors of values for parameter search
    std::vector<double> learning_rates = {.0001, .0005, .001, .005, .01, .05};
    std::vector<int> batch_sizes = {1, 2, 4, 8, 16, 32, 64, 128};
    std::vector<double> dropouts = {0, .1, .2, .3, .4, .5, .6};
    std::vector<int> model_architectures = {1, 2, 3, 4};
    
    // For each batch size
    for (const auto& batch_size_param : batch_sizes) {
        // Creates a dataset and dataloader with appropriate batch size
        auto base_dataset = CustomDataset(training_path, num_inputs, num_outputs);
        const size_t dataset_size = base_dataset.size().value();
        auto mapped_dataset = std::move(base_dataset).map(torch::data::transforms::Stack<>());
        auto data_loader = torch::data::make_data_loader(
            std::move(mapped_dataset),
            torch::data::samplers::RandomSampler(dataset_size),
            torch::data::DataLoaderOptions().batch_size(batch_size_param)
        );

        // For each learning rate
        for (const auto& learning_rate : learning_rates) {
            // For each dropout rate
            for (const auto& dropout : dropouts) {
                // For each neural network architecture
                for (const auto& model_architecture : model_architectures) {
                    // Creates instances of Module, which nets extend
                    std::shared_ptr<torch::nn::Module> net;
                    std::shared_ptr<torch::nn::Module> eval_net;

                    // Initializes network using correct function
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

                    // Declares loss function
                    torch::nn::MSELoss criterion;

                    // Declares optimization function
                    torch::optim::SGD optimizer(net->parameters(), torch::optim::
                        SGDOptions(learning_rate).momentum(0.9));
                    
                    // Declares variables to help with training
                    double least_loss = 0;
                    int epochs_since_least = 0;
                    double validation_loss = 0;
                    int epochs;

                    // Repeats training loop
                    for (size_t epoch = 0; epoch <= 100000; ++epoch) {
                        // For each batch in the data
                        for (auto& batch : *data_loader) {
                            // Zeroes gradients
                            optimizer.zero_grad();

                            // Declares output tensor
                            torch::Tensor output;

                            // Calls forward using appropriate model
                            if (model_architecture == 1) {
                                output = std::static_pointer_cast<Net1>(net)
                                    ->forward(batch.data);
                            } else if (model_architecture == 2) {
                                output = std::static_pointer_cast<Net2>(net)
                                    ->forward(batch.data);
                            } else if (model_architecture == 3) {
                                output = std::static_pointer_cast<Net3>(net)
                                    ->forward(batch.data);
                            } else if (model_architecture == 4) {
                                output = std::static_pointer_cast<Net4>(net)
                                    ->forward(batch.data);
                            }
                            
                            // Calculates loss
                            torch::Tensor loss = criterion(output, batch.target);

                            // Back propogates loss
                            loss.backward();

                            // Calculates new weights and biases with optimizer
                            optimizer.step();
                        }

                        // Switches network to eval and calculates loss over validation set
                        net->eval();
                        torch::NoGradGuard no_grad; 
                        for (auto& batch : *validation_data_loader) {
                            torch::Tensor output;
                            if (model_architecture == 1) {
                                output = std::static_pointer_cast<Net1>(net)
                                    ->forward(batch.data);
                            } else if (model_architecture == 2) {
                                output = std::static_pointer_cast<Net2>(net)
                                    ->forward(batch.data);
                            } else if (model_architecture == 3) {
                                output = std::static_pointer_cast<Net3>(net)
                                    ->forward(batch.data);
                            } else if (model_architecture == 4) {
                                output = std::static_pointer_cast<Net4>(net)
                                    ->forward(batch.data);
                            }
                            validation_loss = criterion(output, batch.target)
                                .item<double>();
                        }

                        // Switches network to training
                        net->train();

                        // If current validation loss less than least or first epoch
                        if ((validation_loss < least_loss) || (epoch == 0)) {
                            torch::save(net, model_path);
                            least_loss = validation_loss;
                            epochs_since_least = 0;
                        }

                        // If 100 epochs have passed with no improvement, training ends
                        else if (++epochs_since_least == 100) {
                            epochs = epoch - 100;
                            break;
                        }
                    }

                    // Loads best model and sets to eval
                    torch::load(eval_net, model_path);
                    eval_net->eval();
                    torch::NoGradGuard no_grad;

                    // Calculates loss over training set using appropriate model
                    double testing_loss;
                    for (auto& batch : *test_data_loader) {
                        torch::Tensor test_output;
                        if (model_architecture == 1) {
                            test_output = std::static_pointer_cast<Net1>(eval_net)
                            ->forward(batch.data);
                        } else if (model_architecture == 2) {
                            test_output = std::static_pointer_cast<Net2>(eval_net)
                            ->forward(batch.data);
                        } else if (model_architecture == 3) {
                            test_output = std::static_pointer_cast<Net3>(eval_net)
                            ->forward(batch.data);
                        } else if (model_architecture == 4) {
                            test_output = std::static_pointer_cast<Net4>(eval_net)
                            ->forward(batch.data);
                        }
                        testing_loss = criterion(test_output, batch.target)
                            .item<double>();
                    }
                    // Writes parameters, training epochs, and loss to file
                    file << batch_size_param << ", " << learning_rate << ", "
                        << dropout << ", " << model_architecture << ", " << epochs
                        << ", " << testing_loss << std::endl;
                }                
            }
        }
    }
    return 0;
}