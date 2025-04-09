classdef MsgMOTOR < Message
    properties (Access = private)
        ChangeEncoder (1,1) logical;
        InvertSpinDir (1,1) logical;
        ChangeSpinDir (1,1) logical;
        InvertEncDir (1,1) logical;
        ChangeEncDir (1,1) logical;
        EncoderValue (1,1) int32;
    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgMOTOR(num)
            arguments
                num (1,1) {mustBeInteger} = 0;
            end
            
            obj@Message(Code.MOTOR, num);
        end
    end

    
    % Getters
    methods (Access = public)
        function index = getIndex(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            index = obj.getNum();
        end

        function ce = getChangeEncoder(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            ce = obj.ChangeEncoder;
        end

        function isd = getInvertSpinDir(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            isd = obj.InvertSpinDir;
        end

        function csd = getChangeSpinDir(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            csd = obj.ChangeSpinDir;
        end
        
        function sd = getSpinDirection(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end
            
            sd = int8(obj.ChangeSpinDir * (1-2*obj.InvertSpinDir));
        end

        function ied = getInvertEncDir(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            ied = obj.InvertEncDir;
        end

        function ced = getChangeEncDir(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            ced = obj.ChangeEncDir;
        end

        function ed = getEncDirection(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            ed = int8(obj.ChangeEncDir * (1-2*obj.InvertEncDir));
        end
        
        function ev = getEncoderValue(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            ev = obj.EncoderValue;
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setIndex(obj, index)
            arguments 
                obj (1,1) MsgMOTOR;
                index (1,1) {mustBeInteger};
            end
            
            res = obj.setNum(index);
        end

        function res = setChangeEncoder(obj, ce)
            arguments
                obj (1,1) MsgMOTOR;
                ce (1,1) logical;
            end
            
            obj.ChangeEncoder = ce;
            res = true;
        end

        function res = setInvertSpinDir(obj, isd)
            arguments
                obj (1,1) MsgMOTOR;
                isd (1,1) logical;
            end

            obj.InvertSpinDir = isd;
            res = true;
        end

        function res = setChangeSpinDir(obj, csd)
            arguments
                obj (1,1) MsgMOTOR;
                csd (1,1) logical;
            end

            obj.ChangeSpinDir = csd;
            res = true;
        end
        
        function res = setSpinDirection(obj, sd)
            arguments
                obj (1,1) MsgMOTOR;
                sd (1,1) {mustBeInteger};
            end

            res = obj.setInvertSpinDir(sd<0) && obj.setChangeSpinDir(sd~=0);
        end

        function res = setInvertEncDir(obj, iec)
            arguments
                obj (1,1) MsgMOTOR;
                iec (1,1) logical;
            end

            obj.InvertEncDir = iec;
            res = true;
        end

        function res = setChangeEncDir(obj, ced)
            arguments
                obj (1,1) MsgMOTOR;
                ced (1,1) logical;
            end

            obj.ChangeEncDir = ced;
            res = true;
        end

        function res = setEncDirection(obj, ed)
            arguments
                obj (1,1) MsgMOTOR;
                ed (1,1) {mustBeInteger};
            end

            res = obj.setInvertSpinDir(ed<0) && obj.setChangeSpinDir(ed~=0);
        end
        
        function res = setEncoderValue(obj, ev)
            arguments
                obj (1,1) MsgMOTOR;
                ev (1,1) {mustBeInteger};
            end

            obj.EncoderValue = int32(ev);
            res = true;
        end
    end
    

    % Data Buffer payload
    methods (Access = protected)
        function dim = bsize_payload(obj) %#ok<MANU>
            arguments
                obj (1,1) MsgMOTOR;
            end

            dim = 5;
        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgMOTOR;
                data (1,:) uint8;
            end

            if(length(data) ~= obj.bsize_payload())
                res = false;
                return;
            end
            
            obj.setChangeEncoder(logical(bitand(data(1), bitshift(1, 0))));
            obj.setInvertSpinDir(logical(bitand(data(1), bitshift(1, 1))));
            obj.setChangeSpinDir(logical(bitand(data(1), bitshift(1, 2))));
            obj.setInvertEncDir( logical(bitand(data(1), bitshift(1, 3))));
            obj.setChangeEncDir( logical(bitand(data(1), bitshift(1, 4))));
            obj.setEncoderValue(Message.bytesToValue(data(2:5), 'int32'));

            res = true;
        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end

            data = zeros([1, obj.bsize_payload()], 'uint8');

            data(1) = data(1) + uint8(bitshift(uint8(obj.getChangeEncoder()), 0));
            data(1) = data(1) + uint8(bitshift(uint8(obj.getInvertSpinDir()), 1));
            data(1) = data(1) + uint8(bitshift(uint8(obj.getChangeSpinDir()), 2));
            data(1) = data(1) + uint8(bitshift(uint8(obj.getInvertEncDir()) , 3));
            data(1) = data(1) + uint8(bitshift(uint8(obj.getChangeEncDir()) , 4));
            data(2:5) = Message.valueToBytes(obj.getEncoderValue(), 'int32');
        end
    end
end