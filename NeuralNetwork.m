classdef NeuralNetwork
    properties
        model;
        weight;
        
    end
    methods
        function model = neural_net(layers, input, output)
            model.input = input;
            model.output = output;
            model.layers = layers;
        end
        function model = layers(layertypes, weights)
            model.layertypes = layertypes;
            model.weights = weights;
        end
    end
end