% Neural Network Initialize parameter
clear;
clc;

V = rand(25,1);
disp(V')
networkLogs = {'';'';'';'';''};
[Wi, Vi, networkLogs{1}] = inputLayer(V,7); % should be just V as a parameter and having Vi as the only output 
[W1, V1, B1, networkLogs{2}] = denseLayer(7, 'relu', Vi, 17); % 
[W2, V2, B2, networkLogs{3}] = denseLayer(17, 'linear', V1, 24);
[W3, V3, B3, networkLogs{4}] = denseLayer(24, 'relu', V2, 4);
[Vo, networkLogs{5}] = outputLayer(4, 'linear', V3);
model = struct('input',Vi,'wi',Wi,'w1',W1,'w2',W2,'w3',W3,'output',Vo);

param = (size(Wi,1)*size(Wi,2)) + (size(W1,1)*size(W1,2)) + (size(W2,1)*size(W2,2)) ...
    + (size(W3,1)*size(W3,2)) + (length(B1)+length(B2)+length(B3));
networkLogs{6} = ['Total parameter : ' num2str(param)];
disp(networkLogs{6})
disp(model)
%% 
inp = rand(7,1);
[w1, b1, v1, acfun] = nn_layer(inp, 3, 'linear');
layer1 = struct('input',inp,'weight',w1,'bias',b1,'output',v1,'activation_function',acfun);
[w2, b2, v2, acfun] = nn_layer(v1, 2, 'sigmoid');
layer2 = struct('input',v1,'weight',w2,'bias',b2,'output',v2,'activation_function',acfun);
[w3, b3, v3, acfun] = nn_layer(v2, 3, 'relu');
layer3 = struct('input',v2,'weight',w3,'bias',b3,'output',v3,'activation_function',acfun);
[w4, b4, v4, acfun] = nn_layer(v3, 7, 'sigmoid');
layer4 = struct('input',v3,'weight',w4,'bias',b4,'output',v4,'activation_function',acfun);
%%
function [model] = neuralNetwork(varargin)
    % Mainly forward propagation and save it into computable struct of NN model
    narginchk(4,10)

    inputValues = varargin(1);
    neuronLayer1 = varargin(3);
    activationFunction1 = varargin(4);
    neuronLayer2 = varargin(5);
    activationFunction2 = varargin(6);
    neuronLayer3 = varargin(7);
    activationFunction3 = varargin(8);
    outputNeuron = varargin(9);
    outputFunction = varargin(10);

    [Wi, Vi, networkLogs(1)] = inputLayer(inputValues, neuronLayer1);
    [W1, V1, B1, networkLogs(2)] = denseLayer(neuronLayer1, activationFunction1, Vi, neuronLayer1);
    [W2, V2, B2, networkLogs(3)] = denseLayer(neuronLayer1, activationFunction2, V1, neuronLayer2);
    [W3, V3, B3, networkLogs(4)] = denseLayer(neuronLayer1, activationFunction3, V2, neuronLayer3);
    [outputValues, networkLogs(5)] = outputLayer(outputNeuron, outputFunction, V3);

end 

function [outputWeight, outputValues, biases, layerInfo] = denseLayer(neuron, actFunc, inputWeight, nOutput)
    % inputWeight is n x n matrix consisted of weights from input neurons
    biases = rand(neuron,1);
    values = zeros(neuron,1);
    outputWeight = rand(neuron, nOutput);
    outputValues = zeros(neuron,nOutput);
    for i = 1:neuron
        values(i) = sum(inputWeight(:,1))+biases(i);
    end
    values = activationFunction(values, actFunc, 'forward');
    
    for i = 1:neuron
        for j = 1:nOutput
            outputValues(i,j) = outputWeight(i,j)*values(i);
        end
    end
    layerInfo = ['Dense Layer with ' int2str(neuron) ' neurons and ' actFunc ' activation function'];
    disp(layerInfo);
end

function [outputWeight, outputValues, layerInfo] = inputLayer(inputValues, nOutput)
    % outputWeight ==> N x M matrix from N input neurons to M output neurons
    nInput = length(inputValues);
    outputWeight = rand(nInput,nOutput);
    outputValues = zeros(nInput,nOutput);
    for i = 1:nInput
        for j = 1:nOutput
            outputValues(i,j) = outputWeight(i,j)*inputValues(i);
        end
    end
    layerInfo = ['Input layer with ' int2str(nInput) ' data' ];   
    disp(layerInfo);
end

function [outputValues, layerInfo] = outputLayer(neuron, actFunc, inputWeight)
    biases = rand(neuron,1);
    values = zeros(neuron,1);
    for i = 1:neuron
        values(i) = sum(inputWeight(:,i))+biases(i);
    end
    outputValues = activationFunction(values, actFunc, 'forward');
    layerInfo = ['Output Layer with ' int2str(neuron) ' data'];
    disp(layerInfo);
end

function [outputValues] = activationFunction(inputValues, actFunc, stepType)
    if strcmpi(stepType,'forward')
        switch actFunc
            case 'linear'
                outputValues = inputValues;
                outputValues = outputValues./sum(outputValues);
            case 'sigmoid'
                outputValues = 1./(1+exp(-inputValues));
                outputValues = outputValues./sum(outputValues);
            case 'tanh'
                outputValues = (exp(inputValues) - exp(-inputValues)) ./ (exp(inputValues) + exp(-inputValues));
                outputValues = outputValues./sum(outputValues);
            case 'relu'
                if inputValues <= 0
                    outputValues = 0;
                else
                    outputValues = inputValues;
                    outputValues = outputValues./sum(outputValues);
                end
        end
    elseif strcmpi(stepType,'back')
        switch actFunc
            case 'linear'
                outputValues = inputValues;
                outputValues = outputValues./sum(outputValues);
            case 'sigmoid'
                outputValues = exp(-inputValues)/(1+exp(-inputValues)).^2;
                outputValues = outputValues./sum(outputValues);
            case 'tanh'
                outputValues = 1- {((exp(inputValues) - exp(-inputValues)).^2) ./ ((exp(inputValues) + exp(-inputValues)).^2)};
                outputValues = outputValues./sum(outputValues);
            case 'relu'
                if inputValues <= 0
                    outputValues = 0;
                else
                    outputValues = 1;
                end
        end
    end
end

function [lossValue] = lossFunction(inputValues, outputValues, lossFunc)
    switch lossFunc
        case 'error'
            lossValue = sum(outputValues - inputValues);
        case 'mse'
            lossValue = sum(outputValues.^2 - inputValues.^2)/length(outputValues);
        case 'rmse'
            lossValue = sqrt(sum(outputValues.^2 - inputValues.^2)/length(outputValues));
        case 'mae'
            lossValue = sum(abs(outputValues) - abs(inputValues))/length(outputValues);
    end
end

function [updatedParams] = backpropagation(output, label, model, learningRate)
    params = model.W1;
    % gradient descent algorithm here
    loss = lossFunction(label, output, model.lossFunc);
    % gradient of output layer
    actFuncGradient = activationFunction(output,'back');
    outputGradient = output'.*(loss*actFuncGradient);
    % gradient of hidden dense layer

    updatedParams = params - learningRate*(outputGradient);
end

function [newModel] = training(inputData, label, model,learningRate)
    % resulting model with new weight
    output = inputData*model;
    newModel = backpropagation(output, label, model, learningRate);
end

%% Python reference
% def gradient_descent(self, x, y, iterations):
%         for i in range(iterations):
%             Xi = x
%             Xj = self.sigmoid(Xi, self.wij)
%             yhat = self.sigmoid(Xj, self.wjk)
%             # gradients for hidden to output weights
%             g_wjk = np.dot(Xj.T, (y - yhat) * self.sigmoid_derivative(Xj, self.wjk))
%             # gradients for input to hidden weights
%             g_wij = np.dot(Xi.T, np.dot((y - yhat) * self.sigmoid_derivative(Xj, self.wjk), self.wjk.T) * self.sigmoid_derivative(Xi, self.wij))
%             # update weights
%             self.wij += g_wij
%             self.wjk += g_wjk

%% Dense Layer

function [weights, biases, output_values, activation_function] = nn_layer(input_values, neuron_num, activation_function_type)
    % Initialize parameters
    weights = weight_generator(length(input_values), neuron_num);
    biases = bias_generator(neuron_num);
    
    % Output calculation
    activation_function = activation_function_type;
    output_values = weights' * input_values + biases;
    output_values = activationFunction(output_values, activation_function, 'forward');
end

function weights = weight_generator(num_input, num_output)
    weights = rand(num_input,num_output);
end

function biases = bias_generator(neuron_num)
    biases = rand(neuron_num, 1);
end