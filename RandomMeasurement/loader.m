classdef loader < handle
   properties
      filename 
   end
   methods
       function obj = loader(saving_path)
         obj.filename = saving_path;
         fileID = fopen(obj.filename, 'w');
         fclose(fileID);
       end

       function append_state(obj, state)
         fileID = fopen(obj.filename, 'a');

         % Append data to the file
        for state_idx = 1:7
            fprintf(fileID, '%d ', state(state_idx));
        end
        fprintf(fileID, '\n');
      
        % Close the file
        fclose(fileID);
       end

      
   end
end