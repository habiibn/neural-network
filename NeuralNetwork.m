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
            end
        end

        function weights = assignParams(obj)
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
            disp(obj.layer)
        end

        function obj = training(obj, inputs, labels, epoch, learningRate, batchSize)
            obj.input = inputs;
            obj.label = labels;
            [tempWeights, tempBiases] = temporaryParameters(obj);
            for i = 1 : epoch
                for j = 1 : length(inputs)
                    obj.output = forwardPropagation(obj, j);
                    [obj.weight, obj.bias] = backPropagation(obj);
                    if j ~= batchSize
                        index = mod(j, batchSize);
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
            for i = 1 : length(fieldnames(obj.weight))
                
            end
        
        end

        function outputs = forwardPropagation(obj, inputs)
            outputs = inputs;
            for i = 1 : length(fieldnames(obj.weight))
                w = ['w',int2str(i)];
                b = ['b',int2str(i)];
                outputs = (outputs'*obj.weight.(w))+obj.bias.(b);
                outputs = activationFunction(outputs, actFunc, 'forward');
            end
        end

        function [weights, biases] = backPropagation(obj)
            % Return gradient of weights and biases 
            weights = obj.weight;
            biases = obj.bias;
            for i = 1 : length(fieldnames(obj.weight))
                w = ['w',int2str(i)];
                outputs = outputs'*obj.weight.(w);
                outputs = activationFunction(outputs, actFunc, 'forward');
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
    end
end