classdef Link < handle
    properties
        startFrame
        endFrame
        lineHandle  % Added graphics handle for the link
    end
    methods
        function obj = Link(startFrame, endFrame)
            obj.startFrame = startFrame;
            obj.endFrame = endFrame;
            obj.lineHandle = [];  % Initialize lineHandle
        end
        
        function display(obj)
            obj.update();
        end
        
        function update(obj)
            if ~isempty(obj.lineHandle) && isvalid(obj.lineHandle)  % Delete old line if it exists
                delete(obj.lineHandle);
            end
            % Draw the new line and save its handle
            obj.lineHandle = plot3([obj.startFrame.position(1), obj.endFrame.position(1)], ...
                  [obj.startFrame.position(2), obj.endFrame.position(2)], ...
                  [obj.startFrame.position(3), obj.endFrame.position(3)], 'k-', 'LineWidth', 5);
        end
    end
end
