function runKeyboardTeleopViaAffGoals
    disp('Using getkey command');
    affgoal_pub=AffGoalPublisher('DRIVING_MANIP_CMD');
    msg.aff_type = 'car';
    msg.aff_uid = 1;
    msg.num_dofs = 2;
    msg.dof_name = {'steering_joint' 'gas_joint'};
    msg.dof_value = [0 0];
    key_up = 30;
    key_left = 28;
    key_right = 29;
    key_down = 31;

    key_w= hex2dec('77');
    key_a = hex2dec('61');
    key_d = hex2dec('64');
    key_s = hex2dec('73');
    key_W= hex2dec('57');
    key_A = hex2dec('41');
    key_D = hex2dec('44');
    key_S = hex2dec('53');
    key_ESC = hex2dec('1B');
    while(1)
        disp('press {ESC or Space} to exit, {arrows, wasd or WASD} to move.')
        CH = getkey;
        if(~strcmp(CH,''))
            if((CH==key_w)||(CH==key_W)||(CH==key_up))
                disp('accelerate');
                msg.dof_value(2) = min(msg.dof_value(2)+0.025,1);
                disp(msg.dof_value(2));
                affgoal_pub.publish(msg);
            elseif((CH==key_a)||(CH==key_A)||(CH==key_left))
                disp('turn left');
                msg.dof_value(1) = max(msg.dof_value(1)+5*(pi/180),-pi);
                disp(msg.dof_value(1)*(180/pi));
                affgoal_pub.publish(msg);
            elseif((CH==key_d)||(CH==key_D)||(CH==key_right))
                disp('turn right');
                msg.dof_value(1) = min(msg.dof_value(1)-5*(pi/180),pi);
                disp(msg.dof_value(1)*(180/pi));
                affgoal_pub.publish(msg);
            elseif((CH==key_s)||(CH==key_S)||(CH==key_down))
                disp('deccelerate');
                msg.dof_value(2) = max(msg.dof_value(2)-0.025,0);
                disp(msg.dof_value(2));
                affgoal_pub.publish(msg);
            elseif((CH==key_ESC)||(CH==32))
               break;
            end
        else
            disp('non-ascii key')
        end
    end
end
  function [ch, tim] = getkey(N,nonascii)

    % GETKEY - get a keypress
    %   CH = GETKEY waits for a single keypress and returns the ASCII code. It
    %   accepts all ascii characters, including backspace (8), space (32),
    %   enter (13), etc, that can be typed on the keyboard.
    %   Non-ascii keys (ctrl, alt, ..) return a NaN. CH is a double.
    %
    %   CH = GETKEY(N) waits for N keypresses and returns their ASCII codes.
    %   GETKEY(1) is the same as GETKEY without arguments.
    %
    %   GETKEY('non-ascii') or GETKEY(N,'non-ascii') uses non-documented
    %   matlab features to return a string describing the key pressed.
    %   In this way keys like ctrl, alt, tab etc. can also distinguished.
    %   The return is a string (when N = 1) or a cell array of strings.
    %
    %   [CH,T] = GETKEY(...) also returns the time between the start of the
    %   function and each keypress. This is, however, not that accurate.
    %
    %   This function is kind of a workaround for getch in C. It uses a modal,
    %   but non-visible window, which does show up in the taskbar.
    %   C-language keywords: KBHIT, KEYPRESS, GETKEY, GETCH
    %
    %   Examples:
    %
    %    fprintf('\nPress any key: ') ;
    %    ch = getkey ;
    %    fprintf('%c\n',ch) ;
    %
    %    fprintf('\nPress the Ctrl-key within 3 presses: ') ;
    %    ch = getkey(3,'non-ascii')
    %    if ismemmber('control', ch),
    %      fprintf('OK\n') ;
    %    else
    %      fprintf(' ... wrong keys ...\n') ;
    %    end
    %
    %  See also INPUT, UIWAIT
    %           GETKEYWAIT (File Exchange)

    % for Matlab 6.5 and higher
    % version 2.0 (jun 2012)
    % author : Jos van der Geest
    % email  : jos@jasen.nl
    %
    % History
    % 1.0 2005 - creation
    % 1.1 dec 2006 - modified lay-out and help
    % 1.2 apr 2009 - tested for more recent MatLab releases
    % 1.3 jan 2012 - modified a few properties, included check is figure still
    %            exists (after comment on FEX by Andrew).
    % 2.0 jun 2012 - added functionality to accept multiple key presses

    t00 = tic ; % start time of this function

    % check the input arguments
    error(nargchk(0,2,nargin))
    switch nargin
        case 0
            nonascii = '' ;
            N = 1 ;
        case 1
            if ischar(N),
                nonascii = N ;
                N = 1 ;
            else
                nonascii = '' ;
            end
    end

    if numel(N) ~= 1 || ~isnumeric(N) || N < 1 || fix(N) ~= N
        error('N should be a positive integer scalar.') ;
    end

    % Determine the callback string to use
    if strcmpi(nonascii,'non-ascii'),
        % non-ascii characters are accepted
        nonascii = true ;
        callstr = 'set(gcbf,''Userdata'',get(gcbf,''Currentkey'')) ; uiresume ' ;
    elseif isempty(nonascii)
        nonascii = false ;
        % only standard ascii characters are accepted
        callstr = 'set(gcbf,''Userdata'',double(get(gcbf,''Currentcharacter''))) ; uiresume ' ;
    else
        error('String argument should be the string ''non-ascii''') ;
    end

    % Set up the figure
    % May be the position property  should be individually tweaked to avoid visibility
    fh = figure(...
        'name','Press a key', ...
        'keypressfcn',callstr, ...
        'windowstyle','modal',...
        'numbertitle','off', ...
        'position',[0 0  1 1],...
        'userdata','timeout') ;
    try
        ch = cell(1,N) ;
        tim = zeros(1,N) ;

        % loop to get N keypresses
        for k=1:N
            % Wait for something to happen, usually a key press so uiresume is
            % executed
            uiwait ;
            tim(k) = toc(t00) ; % get the time of the key press
            ch{k} = get(fh,'Userdata') ;  % and the key itself
            if isempty(ch{k}),
                if nonascii
                    ch{k} = NaN ;
                else
                    ch{k} = '' ;
                end
            end
        end
        if ~nonascii
            ch = [ch{:}] ;
        else
            if N==1
                ch = ch{1} ; % return as a string
            end
            % return as a cell array of strings
        end
     catch
        % Something went wrong, return empty matrices.
        ch = [] ;
        tim = [] ;
    end

    % clean up the figure, if it still exists
    if ishandle(fh)
        delete(fh) ;
    end

end

