classdef Link < handle
    properties
        startFrame
        endFrame
    end
    methods
        function obj = Link(startFrame, endFrame)
            obj.startFrame = startFrame;
            obj.endFrame = endFrame;
        end
        
        function display(obj)
            plot3([obj.startFrame.position(1), obj.endFrame.position(1)], ...
                  [obj.startFrame.position(2), obj.endFrame.position(2)], ...
                  [obj.startFrame.position(3), obj.endFrame.position(3)], 'k-', 'LineWidth', 2.5);
        end
    end
end
