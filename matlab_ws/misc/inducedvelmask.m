classdef inducedvelmask

    methods(Static)

        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle 
        %  - MaskObject 
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.
        function MaskInitialization(maskInitContext)

            MaskWorkspace = maskInitContext.MaskWorkspace;
            VhInit = MaskWorkspace.get('VhInit');
            ConvThresh = MaskWorkspace.get('ConvThresh');


            assignin('base', 'lastVh', VhInit);
            assignin('base', 'convergence_threshold', ConvThresh);
        end

        % Use the code browser on the left to add the callbacks.

    end
end