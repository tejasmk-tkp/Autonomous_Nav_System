from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button

def handle_button_press(instance):
    print(instance.text, instance.center_x, instance.center_y)

class RoverControlApp(App):
    def build(self):
        layout = GridLayout(cols = 3)

        button_size = (150, 150)

        center_button = Button(text = 'Stop', on_press = handle_button_press)
        up_button = Button(text = 'Forward', on_press = handle_button_press)
        down_button = Button(text = 'Backward', on_press = handle_button_press)
        left_button = Button(text = 'Left', on_press = handle_button_press)
        right_button = Button(text = 'Right', on_press = handle_button_press)

        layout.add_widget(center_button, index = 3)
        layout.add_widget(up_button)
        layout.add_widget(down_button)
        layout.add_widget(left_button)
        layout.add_widget(right_button)

        return layout


if __name__ == '__main__':
    RoverControlApp().run()
