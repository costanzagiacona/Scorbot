classdef MsgREF < Message
    properties (Access = private)
        Deltas (1,8) int16;
    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgREF(num)
            arguments
                num (1,1) {mustBeInteger} = 0;
            end
            
            obj@Message(Code.REF, num);
        end
    end

    
    % Getters
    methods (Access = public)
        function count = getCount(obj)
            arguments
                obj (1,1) MsgREF;
            end

            count = obj.getNum() + 1;
        end

        function pwm = getDeltaEnc(obj, index)
            arguments
                obj (1,1) MsgREF;
                index (1,1) {mustBeInteger};
            end
            
            if(index >= 1 && index <= obj.getCount())
                pwm = obj.Deltas(index);
            else
                pwm = int16(0);
            end
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setCount(obj, count)
            arguments 
                obj (1,1) MsgREF;
                count (1,1) {mustBeInteger}
            end
            
            res = obj.setNum(count - 1);
        end

        function res = setDeltaEnc(obj, index, value)
            arguments
                obj (1,1) MsgREF;
                index (1,1) {mustBeInteger};
                value (1,1) {mustBeInteger};
            end

            if(index >= 1 && index <= obj.getCount())
                obj.Deltas(index) = int16(min(max(value, -255), 255));
                res = true;
            else
                res = false;
            end
        end
    end
    

    % Data Buffer payload
    methods (Access = protected)
        function dim = bsize_payload(obj)
            arguments
                obj (1,1) MsgREF;
            end

            dim = 1 + obj.getCount();
        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgREF;
                data (1,:) uint8;
            end
            
            if(length(data) ~= obj.bsize_payload())
                res = false;
                return;
            end

            for k = 1:obj.getCount()
                sgn = 1 - 2 * bitand(data(1), bitshift(1, k-1));
                val = int16(data(1+k));
                obj.setDeltaEnc(k, sgn*val);
            end

            res = true;
        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgREF;
            end

            data = zeros([1, obj.bsize_payload()], 'uint8');

            for k = 1:obj.getCount()
                enc = obj.getDeltaEnc(k);
                data(1) = bitor(data(1), bitshift(uint8(enc<0), k-1));
                data(1+k) = uint8(abs(enc));
            end
        end
    end
end