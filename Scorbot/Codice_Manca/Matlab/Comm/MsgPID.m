classdef MsgPID < Message
    properties (Access = private)
        Div (1,1) single;
        Kp (1,1) single;
        Ki (1,1) single;
        Kd (1,1) single;
        Sat (1,1) single;
        Pole (1,1) single;
    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgPID(num)
            arguments
                num (1,1) {mustBeInteger} = 0;
            end
            
            obj@Message(Code.PID, num);
        end
    end

    
    % Getters
    methods (Access = public)
        function index = getIndex(obj)
            arguments
                obj (1,1) MsgPID;
            end

            index = obj.getNum();
        end

        function div = getPidDiv(obj)
            arguments
                obj (1,1) MsgPID;
            end

            div = obj.Div;
        end
        
        function kp = getPidKp(obj)
            arguments
                obj (1,1) MsgPID;
            end

            kp = obj.Kp;
        end

        function ki = getPidKi(obj)
            arguments
                obj (1,1) MsgPID;
            end

            ki = obj.Ki;
        end

        function kd = getPidKd(obj)
            arguments
                obj (1,1) MsgPID;
            end

            kd = obj.Kd;
        end
        
        function sat = getPidSat(obj)
            arguments
                obj (1,1) MsgPID;
            end

            sat = obj.Sat;
        end

        function pole = getPidPole(obj)
            arguments
                obj (1,1) MsgPID;
            end

            pole = obj.Pole;
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setIndex(obj, index)
            arguments 
                obj (1,1) MsgPID;
                index (1,1) {mustBeInteger};
            end
            
            res = obj.setNum(index);
        end
        
        function res = setPidDiv(obj, div)
            arguments
                obj (1,1) MsgPID;
                div (1,1) {mustBeNumeric}
            end

            obj.Div = div;
            res = true;
        end
        
        function res = setPidKp(obj, kp)
            arguments
                obj (1,1) MsgPID;
                kp (1,1) {mustBeNumeric}
            end

            obj.Kp = kp;
            res = true;
        end

        function res = setPidKi(obj, ki)
            arguments
                obj (1,1) MsgPID;
                ki (1,1) {mustBeNumeric}
            end

            obj.Ki = ki;
            res = true;
        end

        function res = setPidKd(obj, kd)
            arguments
                obj (1,1) MsgPID;
                kd (1,1) {mustBeNumeric}
            end

            obj.Kd = kd;
            res = true;
        end

        function res = setPidSat(obj, sat)
            arguments
                obj (1,1) MsgPID;
                sat (1,1) {mustBeNumeric}
            end

            obj.Sat = sat;
            res = true;
        end

        function res = setPidPole(obj, pole)
            arguments
                obj (1,1) MsgPID;
                pole (1,1) {mustBeNumeric}
            end

            obj.Pole = pole;
            res = true;
        end
    end
    

    % Data Buffer payload
    methods (Access = protected)
        function dim = bsize_payload(obj) %#ok<MANU>
            arguments
                obj (1,1) MsgPID;
            end

            dim = 24;
        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgPID;
                data (1,:) uint8;
            end

            if(length(data) ~= obj.bsize_payload())
                res = false;
                return;
            end

            obj.setPidDiv( Message.bytesToValue(data( 1: 4), 'single'));
            obj.setPidKp(  Message.bytesToValue(data( 5: 8), 'single'));
            obj.setPidKi(  Message.bytesToValue(data( 9:12), 'single'));
            obj.setPidKd(  Message.bytesToValue(data(13:16), 'single'));
            obj.setPidSat( Message.bytesToValue(data(17:20), 'single'));
            obj.setPidPole(Message.bytesToValue(data(21:24), 'single'));
        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgPID;
            end

            data = zeros([1, obj.bsize_payload()], 'uint8');

            data( 1: 4) = Message.valueToBytes(obj.getPidDiv() , 'single');
            data( 5: 8) = Message.valueToBytes(obj.getPidKp()  , 'single');
            data( 9:12) = Message.valueToBytes(obj.getPidKi()  , 'single');
            data(13:16) = Message.valueToBytes(obj.getPidKd()  , 'single');
            data(17:20) = Message.valueToBytes(obj.getPidSat() , 'single');
            data(21:24) = Message.valueToBytes(obj.getPidPole(), 'single');
        end
    end
end