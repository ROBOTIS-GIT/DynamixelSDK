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
            startPos = obj.startFrame.getGlobalPosition;
            endPos = obj.endFrame.getGlobalPosition;
            obj.lineHandle = plot3([startPos(1), endPos(1)], ...
                  [startPos(2), endPos(2)], ...
                  [startPos(3), endPos(3)], 'k-', 'LineWidth', 5);
        end
    end
end
