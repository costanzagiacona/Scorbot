classdef Communication < handle
    properties (Access = private)
        Serial (1,1);
    end


    methods
        function obj = Communication(varargin)
            obj.Serial = serialport(varargin{:});
        end
    end


    methods
        function [res, msg] = rcv(obj, timeout_us)
            arguments
                obj (1,1) Communication;
                timeout_us (1,1) {mustBeInteger, mustBeNonnegative} = 0;
            end
            
            time = tic();
            
            while(true)
                if(obj.Serial.NumBytesAvailable > 0)
                    byte = obj.Serial.read(1, 'uint8');
                    [code, res] = Code.convert(bitshift(byte, -3, 'uint8'));
                    if(res)
                        break;
                    end
                end
                if(timeout_us == 0 || toc(time) * 1e6 > timeout_us)
                    res = false;
                    msg = MsgERROR(0);
                    return;
                end
            end
            
            switch(code)
                case Code.IDLE
                    msg = MsgIDLE();
                case Code.PWM
                    msg = MsgPWM();
                case Code.REF
                    msg = MsgREF();
                case Code.ROBOT
                    msg = MsgROBOT();
                case Code.MOTOR
                    msg = MsgMOTOR();
                case Code.PID
                    msg = MsgPID();
                case Code.ACKC
                    msg = MsgACKC();
                case Code.ACKS
                    msg = MsgACKS();
                case Code.ERROR
                    msg = MsgERROR();
            end
            msg.setNum(bitand(byte, 0b00000111, 'uint8'));

            while(true)
                if((obj.Serial.NumBytesAvailable + 1) >= msg.bsize())
                    if (msg.bsize() > 1)
                        body = obj.Serial.read(msg.bsize()-1, 'uint8');
                    else
                        body = zeros([1,0], 'uint8');
                    end
                    res = msg.parse([byte, body]);
                    return;
                end
                if(timeout_us == 0 || toc(time) * 1e6 > timeout_us)
                    res = false;
                    msg = MsgERROR(0);
                    return;
                end
            end
        end
        
        
        function res = snd(obj, msg)
            arguments
                obj (1,1) Communication;
                msg (1,1) Message;
            end
            
            w = obj.Serial.NumBytesWritten;
            obj.Serial.write(msg.bytes(), 'uint8');
            res = (obj.Serial.NumBytesWritten - w) == msg.bsize();
        end
        

        function flush(obj)
            arguments
                obj (1,1) Communication;
            end
            
            obj.Serial.flush();
        end
    end
end