#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <stdio.h>
#include <array>
#include <fmt/format.h>
#include <chrono>
#include <thread>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

struct Terminal {
    unsigned int width;
    unsigned int height;
};

#ifdef __linux__
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

Terminal getTerminal() {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    return Terminal{
        .width = static_cast<unsigned int>(w.ws_col),
        .height = stati_cast<unsigned int>(w.ws_row),
    };
}

void MoveCursorToStart() {
    std::cout << "\x1B[2J\x1B[H";
}

#define SWITCH_TO_ALT_TERMINAL "\u001B[?1049h" 
#define RESTORE_TERMINAL "\u001B[?1049l" 
#elif _WIN32
#define NOMINMAX
#include <Windows.h>

Terminal getTerminal() {
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    int columns, rows;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
    columns = csbi.srWindow.Right - csbi.srWindow.Left + 1;
    rows = csbi.srWindow.Bottom - csbi.srWindow.Top + 1;
    return Terminal{
        .width = static_cast<unsigned int>(columns),
        .height = static_cast<unsigned int>(rows),
    };
 }

void MoveCursorToStart() {
    static const HANDLE std_handle = GetStdHandle(STD_OUTPUT_HANDLE);
    static const COORD top_left = { 0, 0 };
    SetConsoleCursorPosition(std_handle, top_left);
}
#endif

struct Pixel {
	char colorCode[19];
	char pixel;
};

using iVec2 = std::array<int, 2>;
using iVec3 = std::array<int, 3>;
using Vec2 = std::array<float, 2>;
using Vec3 = std::array<float, 3>;

struct Vertex {
	Vec2 pos;
	Vec3 color;
	Vec2 texPos;
};

// Probably only works with 8 bit png!
struct Texture {
	Texture(const char* path) {
		data = stbi_load(path, &width, &height, &numComponents, 0);
	}

	iVec3 GetPixel(iVec2 pos) const {
		unsigned char* pixelOffset = data + (pos[1] * width + pos[0]) * numComponents;
		unsigned char r = pixelOffset[0];
		unsigned char g = pixelOffset[1];
		unsigned char b = pixelOffset[2];
		return iVec3{r, g, b};
	}

	~Texture() {
		stbi_image_free(data);
	}
	stbi_uc* data = nullptr;
	int width = 0;
	int height = 0;
	int numComponents;
};

Vec2 ndsToPixelCoordinates(Vec2 nds, unsigned short rows,  unsigned short cols) {
	return Vec2 {
		round((nds[0] + 1.0f) * 0.5f * (cols - 1)),
		round((1.0f - nds[1]) * 0.5f * (rows - 1))
	};
}


Vec2 rotate(Vec2 point, Vec2 center, float angle) {
	Vec2 rot;
	point[0] -= center[0];
	point[1] -= center[1];
	rot[0] = point[0] * std::cos(angle) - point[1] * std::sin(angle);
	rot[1] = point[0] * std::sin(angle) + point[1] * std::cos(angle);
	rot[0] += center[0];
	rot[1] += center[1];
	return rot;
}

std::string getColorEscapeCode(Vec3 color) {
	return fmt::format(
		"\x1b[38;2;{:03d};{:03d};{:03d}m",
		static_cast<int>(color[0] * 255),
		static_cast<int>(color[1] * 255),
		static_cast<int>(color[2] * 255)
	);
}

std::string getColorEscapeCode(iVec3 color) {
	return fmt::format(
		"\x1b[38;2;{:03d};{:03d};{:03d}m",		
		color[0],
		color[1],
		color[2]
	);
}

float edge_cross(Vec2 a, Vec2 b, Vec2 p) {
	Vec2 ab = { b[0] - a[0], b[1] - a[1] };
	Vec2 ap = { p[0] - a[0], p[1] - a[1] };
	return ab[0] * ap[1] - ab[1] * ap[0];
}

bool is_top_left(Vec2 start, Vec2 end) {
	Vec2 edge = { end[0] - start[0], end[1] - start[1] };
	bool is_top_edge = edge[1] == 0 && edge[0] > 0;
	bool is_left_edge = edge[1] < 0;
	return is_left_edge || is_top_edge;
}

void writePixel(std::vector<Pixel>& pixels, iVec2 pos, std::string colorCode, Terminal terminal) {
	if (pos[0] < 0 || pos[0] >= terminal.width || pos[1] < 0 || pos[1] >= terminal.height) {
		return;
	}
	std::size_t pxlIdx = (terminal.width * pos[1]) + pos[0];
	memcpy(pixels[pxlIdx].colorCode, colorCode.c_str(), 19);
	pixels[pxlIdx].pixel = '+';
}

void rasterize_triangle(const Texture& texture, std::vector<Pixel>& pixels, Vertex v0, Vertex v1, Vertex v2, Terminal terminal) {	
	Vec2 pos0 = ndsToPixelCoordinates(v0.pos, terminal.height, terminal.width); 
	Vec2 pos1 = ndsToPixelCoordinates(v1.pos, terminal.height, terminal.width);
	Vec2 pos2 = ndsToPixelCoordinates(v2.pos, terminal.height, terminal.width);

	// Finds the bounding= box with all candidate pixels
	int x_min = floor(std::min(std::min(pos0[0], pos1[0]), pos2[0]));
	int y_min = floor(std::min(std::min(pos0[1], pos1[1]), pos2[1]));
	int x_max = ceil(std::max(std::max(pos0[0], pos1[0]), pos2[0]));
	int y_max = ceil(std::max(std::max(pos0[1], pos1[1]), pos2[1]));
	
	// Compute the area of the entire triangle/parallelogram
	float area = edge_cross(pos0, pos1, pos2);

	// Compute the constant delta_s that will be used for the horizontal and vertical steps
	float delta_w0_col = (pos1[1] - pos2[1]);
	float delta_w1_col = (pos2[1] - pos0[1]);
	float delta_w2_col = (pos0[1] - pos1[1]);
	float delta_w0_row = (pos2[0] - pos1[0]);
	float delta_w1_row = (pos0[0] - pos2[0]);
	float delta_w2_row = (pos1[0] - pos0[0]);

	// Rasterization fill rule, not 100% precise due to floating point innacuracy
	float bias0 = is_top_left(pos1, pos2) ? 0 : -0.0001;
	float bias1 = is_top_left(pos2, pos0) ? 0 : -0.0001;
	float bias2 = is_top_left(pos0, pos1) ? 0 : -0.0001;

	// Compute the edge functions for the fist (top-left) point
	Vec2 p0 = { x_min + 0.5f , y_min + 0.5f };
	float w0_row = edge_cross(pos1, pos2, p0) + bias0;
	float w1_row = edge_cross(pos2, pos0, p0) + bias1;
	float w2_row = edge_cross(pos0, pos1, p0) + bias2;

	// Loop all candidate pixels inside the bounding box
	for (int y = y_min; y <=y_max; y++) {
		float w0 = w0_row;
		float w1 = w1_row;
		float w2 = w2_row;
		
		for (int x = x_min; x <= x_max; x++) {
			bool is_inside = w0 <= 0 && w1 <= 0 && w2 <= 0;
			if (is_inside) {
				// Barycentric coordinates
				float alpha = w0 / area;
				float beta  = w1 / area;
				float gamma = w2 / area;
				
				// Interpolate color between vertices
				Vec3 color {
					(alpha) * v0.color[0] + (beta) * v1.color[0] + (gamma) * v2.color[0],
					(alpha) * v0.color[1] + (beta) * v1.color[1] + (gamma) * v2.color[1],
					(alpha) * v0.color[2] + (beta) * v1.color[2] + (gamma) * v2.color[2],
				};

				// Calculate texel position
				iVec2 texelPos {
					static_cast<int>((alpha * v0.texPos[0] + beta * v1.texPos[0] + gamma * v2.texPos[0]) * (texture.width - 1)),
					static_cast<int>((alpha * v0.texPos[1] + beta * v1.texPos[1] + gamma * v2.texPos[1]) * (texture.height - 1))
				};

				// Sample texel
				iVec3 texelColor = texture.GetPixel(texelPos);	
				
				// Combine pixel
				Vec3 finalColor {
					color[0] * (texelColor[0] / 255.0f),
					color[1] * (texelColor[1] / 255.0f),
					color[2] * (texelColor[2] / 255.0f)
				};
				
				// Draw pixel
				writePixel(pixels, iVec2{x, y}, getColorEscapeCode(finalColor), terminal);
			}
			w0 += delta_w0_col;
			w1 += delta_w1_col;
			w2 += delta_w2_col;
		}
		
		w0_row += delta_w0_row;
		w1_row += delta_w1_row;
		w2_row += delta_w2_row;
	}
};


int main() {
	stbi_set_flip_vertically_on_load(1);

	// switch to alternative terminal
#ifdef __linux__
    std::cout << SWITCH_TO_ALT_TERMINAL;
#endif 
    MoveCursorToStart();
	// get terminal dimensions
    Terminal terminal = getTerminal();
	std::size_t pixelCount = terminal.width * terminal.height;
	
	// Our vertices - anticlockwise winding order
	std::array vertices = {
		// triangle 1
		Vertex {
			.pos = {-0.5f, -0.5f},
			.color = {1.0f, 0.0f, 0.0f},
			.texPos = {0.0f, 0.0f},
		},
		Vertex {
			.pos = {0.5f, -0.5f},
			.color = {0.0f, 1.0f, 0.0f},
			.texPos = {1.0f, 0.0f},
		},
		Vertex {
			.pos = {-0.5f, 0.5f},
			.color = {0.0f, 0.0f, 1.0f},
			.texPos = {0.0f, 1.0f}
		},
		Vertex {
			.pos = {0.5f, 0.5f},
			.color = {1.0f, 0.0f, 0.0f},
			.texPos = {1.0f, 1.0f}
		}
	};	

	// Load texture
	Texture texture("awesomeface.png");
	auto a = std::chrono::system_clock::now();
	auto b = std::chrono::system_clock::now();
	double frameTime = (1.0 / 10) * 1000;

	while (true)
    {
        // Maintain designated frequency of 5 Hz (200 ms per frame)
        a = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> work_time = a - b;

        if (work_time.count() < frameTime)
        {
            std::chrono::duration<double, std::milli> delta_ms(frameTime - work_time.count());
            auto delta_ms_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms_duration.count()));
        }

        b = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> sleep_time = b - a;

		// run frame
		// create a pixel buffer
        MoveCursorToStart();
		std::vector<Pixel> pixels(pixelCount);
		for (std::size_t i = 0; i < pixelCount; i++) {
			char color[19] = {'\x1b', '[', '3', '8', ';', '2', ';', '2', '5', '5', ';', '2', '5', '5', ';', '2', '5', '5', 'm'};
			memcpy(pixels[i].colorCode, color, sizeof(color));
			pixels[i].pixel = ' ';
		}

		// rotate triangle
		for (auto& vertex : vertices) {
			vertex.pos = rotate(vertex.pos, Vec2{0.0f, 0.0f}, 0.1f);
		}
	 	
		// rasterize triangles
		rasterize_triangle(texture, pixels, vertices[0], vertices[1], vertices[2], terminal);
		rasterize_triangle(texture, pixels, vertices[3], vertices[2], vertices[1], terminal);	

		// print
		std::vector<char> temp(sizeof(Pixel) * pixels.size() + 1);
		memcpy(temp.data(), pixels.data(), sizeof(Pixel) * pixels.size());
		temp[sizeof(Pixel) * pixels.size()] = '\0';
		std::cout << temp.data();
    }

#ifdef __linux__
	// restore the previous terminal
	std::cout << RESTORE_TERMINAL;	
#endif 
	return 0;
}
