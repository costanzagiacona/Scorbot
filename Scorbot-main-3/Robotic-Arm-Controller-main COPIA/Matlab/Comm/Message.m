classdef Message < handle
    properties (Access = private)
        Hdr (1,1) Header;
    end
    

    % Constructor
    methods (Access = public)
        function obj = Message(code, num)
            arguments
                code (1,1) Code;
                num (1,1) {mustBeInteger} = 0;
            end
            
            obj.Hdr.setCode(code);
            obj.Hdr.setNum(num);
        end
    end
    
    
    % Getters
    methods (Access = public)
        function code = getCode(obj)
            arguments
                obj (1,1) Message;
            end

            code = obj.Hdr.getCode();
        end

        function num = getNum(obj)
            arguments
                obj (1,1) Message;
            end

            num = obj.Hdr.getNum();
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setNum(obj, num)
            arguments
                obj (1,1) Message;
                num (1,1) {mustBeInteger};
            end
            
            res = obj.Hdr.setNum(num);
        end
    end
    

    % Data Buffer
    methods (Access = public)
        function dim = bsize(obj)
            arguments
                obj (1,1) Message;
            end

            dim = obj.bsize_header() + obj.bsize_payload();
        end

        function res = parse(obj, data)
            arguments
                obj (1,1) Message;
                data (1,:) uint8;
            end
            
            if(length(data) ~= obj.bsize())
                res = false;
                return;
            end
            
            if(obj.parse_header(data(1:obj.bsize_header())))
                res = obj.parse_payload(data(1+obj.bsize_header():end));
            else
                res = false;
            end
        end

        function data = bytes(obj)
            arguments
                obj (1,1) Message;
            end
            
            data = zeros([1, obj.bsize()], 'uint8');
            data(:) = [obj.bytes_header(), obj.bytes_payload()];
        end
    end


    % Data Buffer payload
    methods (Access = protected, Abstract)
        dim = bsize_payload(obj)
        
        res = parse_payload(obj, data)

        data = bytes_payload(obj)
    end
    

    % Data Buffer header
    methods (Access = private)
        function dim = bsize_header(obj)
            arguments
                obj (1,1) Message;
            end

            dim = obj.Hdr.bsize();
        end

        function res = parse_header(obj, data)
            arguments
                obj (1,1) Message;
                data (1,:) uint8;
            end
            
            if(length(data) ~= obj.bsize_header())
                res = false;
                return;
            end
            
            tmp = Header();
            res = tmp.parse(data);
            if(res && obj.Hdr.getCode() == tmp.getCode())
                res = obj.Hdr.parse(data);
            else
                res = false;
            end
        end

        function data = bytes_header(obj)
            arguments
                obj (1,1) Message;
            end
            
            data = obj.Hdr.bytes();
        end
    end


    % Utility
    methods (Static, Access = public)
        function data = valueToBytes(value, type)
            arguments
                value (1,1) {mustBeNumericOrLogical}
                type (1,:) {mustBeTextScalar}
            end
            
            [dim, cont] = Message.sizeof(type);            
            data = zeros([1,dim], 'uint8');
            int = typecast(cast(value, type), cont);
            for k = 1:dim
                tmp = cast(bitshift(int, 8*(1-k)), cont);
                data(k) = uint8(bitand(tmp, 255));
            end
        end

        function value = bytesToValue(data, type)
            arguments
                data (1,:) uint8;
                type (1,:) {mustBeTextScalar}
            end

            [dim, cont] = Message.sizeof(type);
            if(length(data) ~= dim)
                error('Wrong data size');
            end
            int = uint32(0);
            for k = 1:4
                tmp = cast(bitshift(cast(data(k), cont), 8*(k-1)), cont);
                int = cast(bitor(int, tmp), cont);
            end
            value = typecast(int, type);
        end

        function [dim, cont] = sizeof(type)
            switch type
              case {'double', 'int64', 'uint64'}
                dim = 8;
                cont = 'uint64';
              case {'single', 'int32', 'uint32'}
                dim = 4;
                cont = 'uint32';
              case {'char', 'int16', 'uint16'}
                dim = 2;
                cont = 'uint16';
              case {'logical', 'int8', 'uint8'}
                dim = 1;
                cont = 'uint8';
              otherwise
                error('Class "%s" is not supported.', type);
            end
        end
    end
end