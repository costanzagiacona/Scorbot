classdef Code < uint8
    enumeration
        IDLE  ( 1)
        PWM   ( 2)
        REF   ( 3)
        ROBOT (16)
        MOTOR (17)
        PID   (18)
        ACKC  (24)
        ACKS  (25)
        ERROR (31)
    end

    methods
        function res = isCtrl(obj)
            arguments
                obj (1,1) Code;
            end

            res = obj == Code.IDLE || obj == Code.PWM || obj == Code.REF;
        end

        function res = isSetup(obj)
            arguments
                obj (1,1) Code;
            end

            res = obj == Code.ROBOT || obj == Code.MOTOR || obj == Code.PID;
        end

        function res = isAck(obj)
            arguments
                obj (1,1) Code;
            end

            res = obj == Code.ACKC || obj == Code.ACKS;
        end

        function res = isError(obj)
            arguments
                obj (1,1) Code;
            end

            res = obj == Code.ERROR;
        end
    end

    methods (Static)
        function [code, res] = convert(value)
            arguments
                value (1,1) {mustBeInteger};
            end

            switch(uint8(value))
                case Code.IDLE
                    code = Code.IDLE;
                    res = true;
                case Code.PWM
                    code = Code.PWM;
                    res = true;
                case Code.REF
                    code = Code.REF;
                    res = true;
                case Code.ROBOT
                    code = Code.ROBOT;
                    res = true;
                case Code.MOTOR
                    code = Code.MOTOR;
                    res = true;
                case Code.PID
                    code = Code.PID;
                    res = true;
                case Code.ACKC
                    code = Code.ACKC;
                    res = true;
                case Code.ACKS
                    code = Code.ACKS;
                    res = true;
                case Code.ERROR
                    code = Code.ERROR;
                    res = true;
                otherwise
                    code = Code.ERROR;
                    res = false;
            end
        end
    end
end