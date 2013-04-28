%%
function cfg = read_config(filename)

% Set up config object
cfg = config_init();

% Read raw file data
fp = fopen(filename,'r');
tx = textscan(fp,'%c','delimiter','','bufsize', 2^20, 'commentstyle', '#');
fclose(fp);
if (numel(tx)>0)
    tx = tx{1}';
end

% Set up context stack
context_stack = {''};

% Loop through text
while (numel(tx) > 0)
    [c1,c2,toks] = regexp(tx,'\s*(\w+)\s*\{|\s*\}|\s*(\w+)=([^;]+);','once');
    if (numel(c1) == 0)
        break;
    end

    new_start = c2(end)+1;

    % TODO: determine keyword type based on content rather than number of matches

    % Start context
    if (size(toks,1) == 1)
        new_context = tx(toks(1,1):toks(1,2));
        context_stack{end+1} = [context_stack{end},new_context,'.'];

    % End context
    elseif (size(toks,1) == 0)
        if (numel(context_stack) == 1)
            error('Extra } detected');
        end
        context_stack = context_stack(1:end-1);

    % Set variable
    elseif (size(toks,1) == 2)
        key = [context_stack{end},tx(toks(1,1):toks(1,2))];
        val_str = tx(toks(2,1):toks(2,2));
        val = value_from_string(val_str);
        cfg = cfg.set(cfg,key,val);
    end

    % Remove already processed part of text
    tx = tx(new_start:end);
end



%%
function cfg = config_init()

cfg.get = @config_get;
cfg.set = @config_set;
cfg.exists = @config_exists;
cfg.get_fields = @config_get_fields;



%%
function val = value_from_string(str)

val = [];
[c1,c2,toks] = regexp(str,'\[(.*)\]','once');
if (numel(c1))
    inside_brackets = str(toks(1,1):toks(1,2));
    if (numel(inside_brackets) == 0)
        return;
    end
    val = textscan(inside_brackets,'%s','delimiter',',');
    val = val{1};
    all_num = true;
    for i = 1:numel(val)
        val{i} = value_from_string(val{i});
        all_num = all_num & isnumeric(val{i});
    end
    if (all_num)
        val = cell2mat(val);
    end
else
    [c1,c2,toks] = regexp(str,'\"(.*)\"','once');
    if (numel(c1))
        inside_quotes = str(toks(1,1):toks(1,2));
        val = inside_quotes;
    else
        num = str2num(str);
        if (numel(num))
            val = num;
        end
    end
end


%%
function str = field_string_from_key(key)

if (numel(key)>0)
    str = ['fld_',strrep(key,'.','.fld_')];
else
    str = '';
end


%%
function flg = config_exists(cfg,key)

field_str = field_string_from_key(key);
fields = {};
if (numel(field_str)>0)
    fields = textscan(field_str,'%s','delimiter','.');
    fields = fields{1};
end
exist_str = 'cfg.data';
flg = true;
for i = 1:numel(fields)
    eval_str = ['cur = isfield(',exist_str,',''',fields{i},''');'];
    eval(eval_str);
    flg = flg & cur;
    exist_str = [exist_str,'.',fields{i}];
end


%%
function cfg = config_set(cfg, key, val)

field_str = field_string_from_key(key);
eval_str = ['cfg.data.',field_str,'=val;'];
eval(eval_str);


%%
function val = config_get(cfg, key)

val = [];
if (~config_exists(cfg,key))
    warning('config:NonExistentKey','Key %s does not exist', key);
    return;
end

field_str = field_string_from_key(key);
if (numel(field_str) > 0)
    eval_str = ['val = cfg.data.',field_str,';'];
else
    eval_str = ['val = cfg.data;'];
end
eval(eval_str);

if (isstruct(val))
    cfg = config_init();
    cfg.data = val;
    val = cfg;
end


%%
function fields = config_get_fields(cfg,key)

if (nargin < 2)
    key = '';
end

if (~config_exists(cfg,key))
    warning('config:NonExistentKey','Key %s does not exist', key);
    return;
end

field_str = field_string_from_key(key);
if (numel(field_str) > 0)
    eval_str = ['val=cfg.data.',field_str,';'];
    eval(eval_str);
else
    val = cfg.data;
end
fields = fieldnames(val);
for i = 1:numel(fields)
    fields{i} = fields{i}(5:end);
end

