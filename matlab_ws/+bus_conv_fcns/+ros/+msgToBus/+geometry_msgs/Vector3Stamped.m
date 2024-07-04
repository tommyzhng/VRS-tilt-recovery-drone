function slBusOut = Vector3Stamped(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    currentlength = length(slBusOut.Vector_);
    for iter=1:currentlength
        slBusOut.Vector_(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Vector(iter),slBusOut(1).Vector_(iter),varargin{:});
    end
    slBusOut.Vector_ = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Vector,slBusOut(1).Vector_,varargin{:});
end
