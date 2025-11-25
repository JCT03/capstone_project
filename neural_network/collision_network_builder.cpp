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

// Declares neural network
struct Net : torch::nn::Module {
  Net(const int& num_inputs, const int& num_outputs) {
    int middle = (num_inputs+num_outputs)/2;
    fc1 = register_module("fc1", torch::nn::Linear(num_inputs, middle));
    dropout = register_module("dropout", torch::nn::Dropout(0.3));
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

int main() {
    // Paths to data and to save model and outputs
    std::string training_path = 
        "/Users/jacobcollier-tenison/GitHub/capstone_project/collision_data/raw/
        training.csv";
    std::string validation_path = 
        "/Users/jacobcollier-tenison/GitHub/capstone_project/collision_data/raw/
        validation.csv";
    std::string testing_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/
        collision_data/raw/testing.csv";
    std::string model_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/
        networks/1/raw/net.pt";
    std::string output_path = "/Users/jacobcollier-tenison/GitHub/capstone_project/
        output_data/1/raw/output.csv";
    
    // Declares and initializes learning rate and batch size
    double learning_rate = 0.001;
    int batch_size = 32;

    // Declares and initialized number of inputs and outputs to network
    int num_inputs = 22;
    int num_outputs = 3;

    // Creates CustomDataset with training data and links to data loader
    auto base_dataset = CustomDataset(training_path, num_inputs, num_outputs);
    const size_t dataset_size = base_dataset.size().value();
    auto mapped_dataset = std::move(base_dataset)
        .map(torch::data::transforms::Stack<>());
    auto data_loader = torch::data::make_data_loader(
        std::move(mapped_dataset),
        torch::data::samplers::RandomSampler(dataset_size), // Samples in random order
        torch::data::DataLoaderOptions().batch_size(batch_size)
    );

    // Creates CustomDataset with validation data and links to data loader
    auto base_validation_dataset = CustomDataset(validation_path, num_inputs,
        num_outputs);
    const size_t validation_dataset_size = base_validation_dataset.size().value();
    auto mapped_validation_dataset = std::move(base_validation_dataset)
        .map(torch::data::transforms::Stack<>());
    auto validation_data_loader = torch::data::make_data_loader(
        std::move(mapped_validation_dataset),
        torch::data::samplers::SequentialSampler(validation_dataset_size), 
        // Samples in sequential order
        torch::data::DataLoaderOptions().batch_size(validation_dataset_size)
    );

    // Creates instance of neural network
    auto net = std::make_shared<Net>(num_inputs, num_outputs);

    // Declares Mean Squared loss
    torch::nn::MSELoss criterion;

    // Declares ADAM optimizer
    torch::optim::Adam optimizer(net->parameters(), learning_rate);

    // Declares with local variables to help with training
    double least_loss = 0;
    int epochs_since_least = 0;
    double validation_loss = 0;

    // Training loop
    for (size_t epoch = 0; epoch <= 100000; ++epoch) {
        // For each batch of data from the data loader
        for (auto& batch : *data_loader) {
            // Zeroes gradients
            optimizer.zero_grad();
        
            // Gets model outputs from batch
            torch::Tensor output = net->forward(batch.data);

            // Calculates loss
            torch::Tensor loss = criterion(output, batch.target);

            // Back-propogates loss
            loss.backward();

            // Updates weights and biases with optimizer
            optimizer.step();
        }
        // Puts network in eval mode for efficiency and prevents gradient calculations
        net->eval();
        torch::NoGradGuard no_grad; 

        // Calculates loss over validation set
        for (auto& batch : *validation_data_loader) {
            torch::Tensor output = net->forward(batch.data);
            validation_loss = criterion(output, batch.target).item<double>();
        }

        // Puts network back in training mode
        net->train(); 
        
        // If this is the first training epoch or the most recent loss is less than least
        if ((validation_loss < least_loss) || (epoch == 0)) {
            // Saves model and least loss
            torch::save(net, model_path);
            least_loss = validation_loss;

            // Resets number of epochs since least loss
            epochs_since_least = 0;
        }
        // Else if 100 epochs have passed since least loss
        else if (++epochs_since_least == 100) {
                // Outputs number of training epochs and breaks training loop
                std::cout << "trained for " << epoch - 100 << " epochs" << std::endl;
                break;
        }
    }

    // Loads back best-performing network on validation set
    auto eval_net = std::make_shared<Net>(num_inputs, num_outputs);
    torch::load(eval_net, model_path);

    // Sets netowrk to eval and prevents gradients from being calulated
    eval_net->eval();
    torch::NoGradGuard no_grad;

    // Creates dataset and loader from testing data
    auto base_test_dataset = CustomDataset(testing_path, num_inputs, num_outputs);
    const size_t test_dataset_size = base_test_dataset.size().value();
    auto mapped_test_dataset = std::move(base_test_dataset).map(torch::data::transforms::
        Stack<>());
    auto test_data_loader = torch::data::make_data_loader(
        std::move(mapped_test_dataset),
        torch::data::samplers::SequentialSampler(test_dataset_size),
        torch::data::DataLoaderOptions().batch_size(test_dataset_size)
    );

    // Determines outputs for each input in testing set and writes to file
    for (auto& batch : *test_data_loader) {
        torch::Tensor test_output = eval_net->forward(batch.data);
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