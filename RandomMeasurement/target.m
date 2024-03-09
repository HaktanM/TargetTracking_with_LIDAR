classdef target < handle
   properties
      target_shape
      center
      rot = zeros(2,2)
   end
   methods
       function obj = target(shape_matrix,center_in_meters,heading_in_rad)
         obj.target_shape = shape_matrix;
         obj.center = center_in_meters;
         obj.rot(1,1) = cos(heading_in_rad);
         obj.rot(1,2) = -sin(heading_in_rad);
         obj.rot(2,1) = sin(heading_in_rad);
         obj.rot(2,2) = cos(heading_in_rad);
       end

       function modifyPose(obj,new_center,new_heading)
         obj.center = new_center;
         obj.rot(1,1) = cos(new_heading);
         obj.rot(1,2) = -sin(new_heading);
         obj.rot(2,1) = sin(new_heading);
         obj.rot(2,2) = cos(new_heading);
       end

       function resizeShape(obj, scaleM)
           obj.target_shape = obj.target_shape * scaleM;
       end

       function boundary = getBoundary(obj)
           boundary = obj.target_shape * transpose(obj.rot);
           boundary(:,1) = boundary(:,1) + obj.center(1);
           boundary(:,2) = boundary(:,2) + obj.center(2);
       end
   end
end