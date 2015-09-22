debug_print("Window Name: " .. get_window_name());
debug_print("Application name: " .. get_application_name())

if (get_window_name()=="Projector Calibration") then
	set_window_position(1920,0)
end

if (get_window_name()=="object_circler.py") then
	set_window_position(1920,0)
end

if (get_application_name()=="VLC media player") then
	set_window_position(0,0);
	maximize();
end

if (get_window_name()=="splashscreen.py") then
    set_window_position(0,0)
    set_on_top()
end

if (get_window_name()=="mouse_capture.py") then
    set_on_top()
end

