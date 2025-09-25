classdef Header < handle
    properties (Access = private)
        Cod (1,1) Code = Code.IDLE;
        Num (1,1) uint8;
    end

    
    % Constructor
    methods (Access = public)
        function obj = Header(code, num)
            arguments
                code (1,1) Code = Code.IDLE;
                num (1,1) {mustBeInteger} = 0;
            end

            obj.setCode(code);
            obj.setNum(num);
        end
    end
    
    
    % Getters
    methods (Access = public)
        function code = getCode(obj)
            arguments
                obj (1,1) Header;
            end

            code = obj.Cod;
        end

        function num = getNum(obj)
            arguments
                obj (1,1) Header;
            end

            num = obj.Num;
        end

        function count = getCount(obj)
            arguments
                obj (1,1) Header;
            end

            count = obj.getNum() + 1;
        end

        function index = getIndex(obj)
            arguments
                obj (1,1) Header;
            end

            index = obj.getNum();
        end
    end
    
    
    % Setters
    methods (Access = public)
        function res = setCode(obj, code)
            arguments
                obj (1,1) Header;
                code (1,1) Code;
            end
            
            obj.Cod = code;
            res = true;
        end

        function res = setNum(obj, num)
            arguments
                obj (1,1) Header;
                num (1,1) {mustBeInteger};
            end
            
            if(num >= 0 && num <= 7)
                obj.Num = uint8(num);
                res = true;
            else
                res = false;
            end
        end

        function res = setCount(obj, count)
            arguments
                obj (1,1) Header;
                count (1,1) {mustBeInteger};
            end
            
            res = obj.setNum(count - 1);
        end

        function res = setIndex(obj, index)
            arguments
                obj (1,1) Header;
                index (1,1) {mustBeInteger};
            end
            
            res = obj.setNum(index);
        end
    end


    % Data Buffer
    methods (Access = public)
        function dim = bsize(obj) %#ok<MANU>
            arguments
                obj (1,1) Header;
            end

            dim = 1;
        end

        function res = parse(obj, data)
            arguments
                obj (1,1) Header;
                data (1,:) uint8;
            end

            [code, res] = Code.convert(bitshift(data(1), -3, 'uint8'));
            if(res)
                obj.Cod = code;
                obj.Num = bitand(data(1), 0b00000111, 'uint8');
            end
        end

        function data = bytes(obj)
            arguments
                obj (1,1) Header;
            end

            data = zeros([1,1], 'uint8');
            data(1) = bitor(bitshift(obj.Cod, 3, 'uint8'), obj.Num, 'uint8');
        end
    end
end