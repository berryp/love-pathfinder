# Path finder module for LÖVE

This library was vibe-coded with Claude Sonnet for my own educational purposes.
I created this as I wasn't satisfied with any of the available packages for LÖVE.

I first had Claude translate the JavaScript code from [this article on A\* search algorithm](https://www.geeksforgeeks.org/a-search-algorithm/) to Lua.
I chose JavaScript as the base as it has the most similarities to Lua when it comes to translation.
I went through several iterations to incrementally improve the algorithm and example code.

I used [T3.chat](https://t3.chat) Pro with Claude 3.7 Sonnet for the translation.
Claude 4 Sonnet was used for the improvements since T3.chat pushed an beta update supporting it.

Overall I am very happy with the results and the experience using Claude and T3.chat.
Though, I can't speak for the quality or reliability of the path finding algorithm.

## Demo

![demo.mp4](https://github.com/user-attachments/assets/085000f6-ce9e-45bb-95ba-8abecec4dbd4)

## Example usage

First install [LÖVE](https://love2d.org/#download).

Then run the game:

```sh
cd example
love .
```
