% Neural Network
% Initialize parameter

clear;
clc;
V = rand(25,1);
disp(V')
networkLogs = {'';'';'';'';''};
[Wi, Vi, networkLogs{1}] = inputLayer(V,7);
[W1, V1, B1, networkLogs{2}] = denseLayer(7, 'relu', Vi, 17);
[W2, V2, B2, networkLogs{3}] = denseLayer(17, 'linear', V1, 24);
[W3, V3, B3, networkLogs{4}] = denseLayer(24, 'relu', V2, 4);
[Vo, networkLogs{5}] = outputLayer(4, 'linear', V3);
param = (size(Wi,1)*size(Wi,2)) + (size(W1,1)*size(W1,2)) + (size(W2,1)*size(W2,2)) ...
    + (size(W3,1)*size(W3,2)) + (length(B1)+length(B2)+length(B3));
networkLogs{6} = ['Total parameter : ' num2str(param)];
disp(networkLogs{6})
disp(Vo')

% function [outputValues, networkLogs] = neuralNetwork(varargin)
%     input ==> input values, N output, layer's neuron
%     narginchk(4,10)
% 
%     inputValues = varargin(1);
%     neuronLayer1 = varargin(3);
%     activationFunction1 = varargin(4);
%     neuronLayer2 = varargin(5);
%     activationFunction2 = varargin(6);
%     neuronLayer3 = varargin(7);
%     activationFunction3 = varargin(8);
%     outputNeuron = varargin(9);
%     outputFunction = varargin(10);
% 
%     [Wi, Vi, networkLogs(1)] = inputLayer(inputValues, neuronLayer1);
%     [W1, V1, B1, networkLogs(2)] = denseLayer(neuronLayer1, activationFunction1, Vi, neuronLayer1);
%     [W2, V2, B2, networkLogs(3)] = denseLayer(neuronLayer1, activationFunction2, V1, neuronLayer2);
%     [W3, V3, B3, networkLogs(4)] = denseLayer(neuronLayer1, activationFunction3, V2, neuronLayer3);
%     [outputValues, networkLogs(5)] = outputLayer(outputNeuron, outputFunction, V3);
% 
% end 

function [outputWeight, outputValues, biases, layerInfo] = denseLayer(neuron, actFunc, inputWeight, nOutput)
    % inputWeight is n x n matrix consisted of weights from input neurons
    biases = rand(neuron,1);
    values = zeros(neuron,1);
    outputWeight = rand(neuron, nOutput);
    outputValues = zeros(neuron,nOutput);
    for i = 1:neuron
        values(i) = sum(inputWeight(:,1))+biases(i);
    end
    values = activationFunction(values, actFunc);
    
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
    outputValues = activationFunction(values, actFunc);
    layerInfo = ['Output Layer with ' int2str(neuron) ' data'];
    disp(layerInfo);
end

function [outputValues] = activationFunction(inputValues, actFunc)
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
end
