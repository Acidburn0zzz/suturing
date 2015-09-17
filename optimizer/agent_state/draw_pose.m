function [  ] = draw_pose( r , scale, color)
% Draws an arrow for a pose in a given color (copy of draw frame)
  if 1 == nargin,
    scale = 1;
  end
  
  p = r(1:3,4);
  r = r(1:3,1:3);

  plot3(p(1), p(2), p(3));
  
  hchek = ishold;
  hold on
  
  arrow3(p, scale*r(1:3,1), color, 3);
%   arrow3(p, 0.5*scale*r(1:3,2), 0.75*color, 2);
%   arrow3(p, 0.5*scale*r(1:3,3), 0.5*color, 2);
%   arrow3(p, scale*r(1:3,2), 'g');
%   arrow3(p, scale*r(1:3,3), 'b');

  xlabel('x');
  ylabel('y');
  zlabel('z');
  
  if hchek == 0
     hold off
  end

end

