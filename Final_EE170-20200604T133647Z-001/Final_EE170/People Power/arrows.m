g = figure('KeyPressFcn', @keyPress)
MyButton = uicontrol('Style', 'pushbutton','Callback',@task);

      function task(src, e)
         disp('button press');
      end

      function keyPress(src, e)
         switch e.Key
             case 'c'
                 task(MyButton, []);
         end
      end