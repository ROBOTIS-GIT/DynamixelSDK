classdef Link < handle
    % The Link class defines a link object in a robot manipulator model.
    % Each link is defined by two frames, a start frame and an end frame.
    % It also maintains a handle to the graphical representation of the link,
    % allowing for the link to be updated or removed from a plot. The link
    % provides methods for creating, displaying, and updating itself.
    properties
        % The startFrame property refers to the starting frame of the link.
        startFrame
        % The endFrame property refers to the ending frame of the link.
        endFrame
        % The lineHandle property stores the graphics handle for the link
        % which can be used to delete the link from the plot when necessary.
        lineHandle
    end
    
    methods
        % This is the constructor of the Link class. It initializes the
        % startFrame and endFrame properties with the frames provided as
        % arguments. It also initializes the lineHandle property to an empty
        % array, as there is no line drawn initially.
        function obj = Link(startFrame, endFrame)
            obj.startFrame = startFrame;
            obj.endFrame = endFrame;
            obj.lineHandle = [];
        end
        
        % The display method updates the display of the link. Currently, it
        % just calls the update method to redraw the link.
        function display(obj)
            obj.update();
        end
        
        % The update method redraws the link. If there was a previous line
        % drawn (i.e., lineHandle is not empty and is valid), it deletes
        % the old line from the plot before drawing the new one. The new
        % line is drawn from the global position of the startFrame to the
        % global position of the endFrame, and its handle is saved in
        % lineHandle for future reference.
        function update(obj)
            if ~isempty(obj.lineHandle) && isvalid(obj.lineHandle)  % Delete old line if it exists
                delete(obj.lineHandle);
            end
            % Draw the new line and save its handle
            startPos = obj.startFrame.getGlobalPosition;
            endPos = obj.endFrame.getGlobalPosition;
            obj.lineHandle = plot3([startPos(1), endPos(1)], ...
                  [startPos(2), endPos(2)], ...
                  [startPos(3), endPos(3)], '-k', 'LineWidth', 5);
        end
    end
end
