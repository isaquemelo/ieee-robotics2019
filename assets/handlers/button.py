#!/usr/bin/env python3
import ev3dev as ev3


class ButtonApproach:
    def __init__(self):
        self.btn = ev3.Button()

    def left(state):
        if state:
            print('Left button pressed')
        else:
            print('Left button released')

    def right(state):  # neater use of 'if' follows:
        print('Right button pressed' if state else 'Right button released')

    def up(state):
        print('Up button pressed' if state else 'Up button released')

    def down(state):
        print('Down button pressed' if state else 'Down button released')

    def enter(state):
        print('Enter button pressed' if state else 'Enter button released')

    def backspace(state):
        print('Backspace button pressed' if state else 'Backspace button released')

    def action_dealer(self):
        self.btn.on_left = self.left
        self.btn.on_right = self.right
        self.btn.on_up = self.up
        self.btn.on_down = self.down
        self.btn.on_enter = self.enter
        self.btn.on_backspace = self.backspace

