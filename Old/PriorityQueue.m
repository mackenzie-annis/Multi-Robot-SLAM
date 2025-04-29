classdef PriorityQueue < handle
    properties
        elements
        priorities
    end
    methods
        function obj = PriorityQueue()
            obj.elements = {};
            obj.priorities = [];
        end
        function insert(obj, element, priority)
            obj.elements{end+1} = element;
            obj.priorities(end+1) = priority;
        end
        function elem = pop(obj)
            [~, idx] = min(obj.priorities);
            elem = obj.elements{idx};
            obj.elements(idx) = [];
            obj.priorities(idx) = [];
        end
        function tf = isEmpty(obj)
            tf = isempty(obj.elements);
        end
    end
end
