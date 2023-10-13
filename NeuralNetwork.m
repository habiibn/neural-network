classdef NeuralNetwork
    properties
        weight;
        bias;
        layer;
        input;
        label;
        output;
        performance;
        summary;
    end
    methods
        function obj = NeuralNetwork(layers)
%             obj.input = input;
%             obj.output = output;
            obj.layer = layers;
%             obj.weight = layers;

            obj.layer = parseLayer(obj);
            obj.weight = assignParams(obj);
        end

        function layers = parseLayer(obj)
            for i = 1:length(obj.layer)
                w = ['layer',int2str(i)];
                layers.(w) = obj.layer(i);
%                 layers.(w).neuron = obj.layer(i);
            end
        end

        function weights = assignParams(obj)
            % TODO: get parsed layer and assign weights and biases into obj
            weights.('wi') = obj.layer.layer1; % first layer is input layer
            for i = 2:length(fieldnames(obj.layer))
                if length(fieldnames(obj.layer)) == i
                    w = 'wo';
                else
                    w = ['w',int2str(i-1)];
                end
                l = ['layer',int2str(i)];
                weights.(w) = obj.layer.(l);
            end
        end

        function displayNetwork(obj)
            % TODO: create summary of network and display it
            disp(obj.layer)
        end

        function obj = training(obj, inputs, labels, epoch, learningRate, batchSize)
            % TODO: parsing input parameyer, complete backpropagation and update parameter
            obj.input = inputs;
            obj.label = labels;
            [tempWeights, tempBiases] = temporaryParameters(obj);
            % Train network
            for i = 1 : epoch
                % Iterate through all inputs with known label
                for j = 1 : length(inputs)
                    obj.output = forwardPropagation(obj, j);
                    [obj.weight, obj.bias] = backPropagation(obj);
                    % Update parameter every batch
                    index = mod(j, batchSize);
                    if index ~= 0
                        tempWeights(index) = obj.weight;
                        tempBiases(index) = obj.bias;
                    else
                        % Optimization used to update parameter
                        obj.weight = obj.weight - (learningRate * sum(obj.weight + sum(tempWeights)));
                        obj.bias = obj.bias - (learningRate * sum(obj.bias + sum(tempBiases)));
                    end
                end
            end
        end
        
        function [tempWeight, tempBias] = temporaryParameters(obj)
            % This function is being used for saving temporary parameter
            % required to be updated later
            for i = 1 : length(fieldnames(obj.weight))
                tempWeight = obj.weight;
                tempBias = obj.bias;
            end
        
        end

        function outputs = forwardPropagation(obj, inputs)
            % Forward propagation for one pair of input and label,
            % resulting output

            outputs = inputs;
            for i = 1 : length(fieldnames(obj.weight))
                w = ['w',int2str(i)];
                b = ['b',int2str(i)];
                outputs = (outputs'*obj.weight.(w))+obj.bias.(b);
                outputs = activationFunction(outputs, actFunc, 'forward');
            end
        end

        function [dW, dB] = backPropagation(obj)
            % Return gradient of weights and biases 
            % TODO: get gradient from layer end to layer input
            dW = obj.weight;
            dB = obj.bias;
            dA_prev = obj.output;
            nLayer = length(fieldnames(obj.weight));

            for i = 0 : nLayer-1
                w = ['w',int2str(nLayer - i)];
                b = ['w',int2str(nLayer - i)];
                actFunc = ['Layer',int2str(nLayer - i)];

                dA_curr = dA_prev;
                dA = dA'*obj.weight.(w);
                dA = activationFunction(dA, actFunc, 'back');
            end
        end

        function outputValues = activationFunction(inputValues, actFunc, stepType)
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

        function lossValue = lossFunction(outputValues, inputValues)
            switch lossFunc
                case 'error'
                    lossValue = sum(outputValues - inputValues);
                case 'mse'
                    lossValue = sum((outputValues - inputValues).^2) / length(outputValues);
                case 'rmse'
                    lossValue = sqrt(sum((outputValues - inputValues).^2) / length(outputValues));
                case 'mae'
                    lossValue = sum(abs(outputValues - inputValues)) / length(outputValues);
            end
        end

    end
end