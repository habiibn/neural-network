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
            obj.layer = setLayers(obj);
        end
        
        function model = addWeight(layertypes, weights)
            model.layertypes = layertypes;
            model.weights = weights;
        end

        function layers = setLayers(obj)
            for i = 1:length(obj.layer)
                w = ['w',int2str(i)];
                layers.(w) = obj.layer(i);
            end
        end
    end
end