classdef MsgACKC < Message
    properties (Access = private)
        EndStops (1,8) logical;
        Deltas (1,8) int16;
    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgACKC(num)
            arguments
                num (1,1) {mustBeInteger} = 0;
            end
            
            obj@Message(Code.ACKC, num);
        end
    end

    
    % Getters
    methods (Access = public)
        function count = getCount(obj)
            arguments
                obj (1,1) MsgACKC;
            end

            count = obj.getNum() + 1;
        end

        function es = getEndStop(obj, index)
            arguments
                obj (1,1) MsgACKC;
                index (1,1) {mustBeInteger};
            end

            if(index >= 1 && index <= obj.getCount())
                es = obj.EndStops(index);
            else
                es = false;
            end
        end

        function pwm = getDeltaEnc(obj, index)
            arguments
                obj (1,1) MsgACKC;
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
                obj (1,1) MsgACKC;
                count (1,1) {mustBeInteger};
            end
            
            res = obj.setNum(count - 1);
        end
        
        function res = setEndStops(obj, index, value)
            arguments
                obj (1,1) MsgACKC;
                index (1,1) {mustBeInteger};
                value (1,1) logical;
            end

            if(index >= 1 && index <= obj.getCount())
                obj.EndStops(index) = value;
                res = true;
            else
                res = false;
            end
        end
        
        function res = setDeltaEnc(obj, index, value)
            arguments
                obj (1,1) MsgACKC;
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
                obj (1,1) MsgACKC;
            end

            dim = 2 + obj.getCount();
        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgACKC;
                data (1,:) uint8;
            end
            
            if(length(data) ~= obj.bsize_payload())
                res = false;
                return;
            end

            for k = 1:obj.getCount()
                ens = logical(bitand(data(1), bitshift(1, k-1, 'uint8') , 'uint8'));
                sgn = 1 - 2 * (bitand(data(2), bitshift(1, k-1, 'uint8') , 'uint8') > 0);
                val = int16(data(2+k));
                obj.setEndStops(k, ens);
                obj.setDeltaEnc(k, sgn*val);
            end

            res = true;
        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgACKC;
            end

            data = zeros([1, obj.bsize_payload()], 'uint8');

            for k = 1:obj.getCount()
                ens = obj.getEndStops(k);
                pwm = obj.getDeltaEnc(k);
                data(1) = bitor(data(1), bitshift(uint8(ens), k-1, 'uint8'), 'uint8');
                data(2) = bitor(data(2), bitshift(uint8(pwm<0), k-1, 'uint8'), 'uint8');
                data(2+k) = uint8(abs(pwm));
            end
        end
    end
end