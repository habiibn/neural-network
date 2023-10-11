classdef NeuralNetwork
    properties
        model;
        weight;
        layer;
        input;
        output;
    end
    methods
        function obj = NeuralNetwork(layers, input, output)
            obj.input = input;
            obj.output = output;
            obj.layer = layers;
        end
        function model = layers(layertypes, weights)
            model.layertypes = layertypes;
            model.weights = weights;
        end
    end
end