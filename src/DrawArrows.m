function  DrawArrows(obj, x, y, lineColor)
% A self-adapatable method of drawing arrows in matlab
% @Params:
%       obj: axis object
%       x: X cordinate of begin and end point [x_begin, x_end]
%       y: Y cordinate of begin and end point [y_begin, y_end]

    
    pos = obj.Position;  % axis's position in figure
    pos_x = pos(1,1);
    pos_y = pos(1,2);
    width  = pos(1,3);
    height = pos(1,4);
    
    xmin = obj.XLim(1,1);
    ymin = obj.YLim(1,1);
    xmax = obj.XLim(1,2);
    ymax = obj.YLim(1,2);
    
    xNew = pos_x + (x - xmin)/(xmax-xmin)*width;
    yNew = pos_y + (y - ymin)/(ymax-ymin)*height;
    
    a = annotation('arrow', xNew,yNew, 'HeadStyle', 'vback3', 'Color', lineColor);
end