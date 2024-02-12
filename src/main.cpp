#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <array>
#include <fmt/format.h>
#include <chrono>
#include <thread>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define WIPE_TERMINAL "\x1B[2J\x1B[H"
#define SWITCH_TO_ALT_TERMINAL "\u001B[?1049h" 
#define RESTORE_TERMINAL "\u001B[?1049l" 

const wchar_t onPixel = L'█';

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
		if (!data) {
			stbi_image_free(data);
			std::cout << fmt::format("Failed to load image {}", path);
		}
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

iVec2 ndsToPixelCoordinates(Vec2 nds, unsigned short rows,  unsigned short cols) {
	return iVec2 {
		static_cast<int>((nds[0] + 1.0f) * 0.5f * (cols - 1)),
		static_cast<int>((1.0f - nds[1]) * 0.5f * (rows - 1))
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

int edge_cross(iVec2 a, iVec2 b, iVec2 p) {
	iVec2 ab {b[0] - a[0], b[1] - a[1]};
	iVec2 ap {p[0] - a[0], p[1] - a[1]};
	return ab[0] * ap[1] - ab[1] * ap[0];
}

bool is_flat_top_or_left_edge(iVec2 start, iVec2 end) {
	iVec2 edge {end[0] - start[0], end[1] - start[1]};
	bool is_flat_top = edge[1] == 0 && edge[0] > 0;
	bool is_left_edge = edge[1] < 0;

	return is_flat_top || is_left_edge;
}

void rasterize_triangle(const Texture& texture, std::vector<Pixel>& pixels, Vertex v0, Vertex v1, Vertex v2, unsigned short rows, unsigned short cols) {
	iVec2 pos0 = ndsToPixelCoordinates(v0.pos, rows, cols);
	iVec2 pos1 = ndsToPixelCoordinates(v1.pos, rows, cols);
	iVec2 pos2 = ndsToPixelCoordinates(v2.pos, rows, cols);

	int delta_w0_col = pos0[1] - pos1[1];
	int delta_w1_col = pos1[1] - pos2[1];
	int delta_w2_col = pos2[1] - pos0[1];

	int delta_w0_row = pos1[0] - pos0[0];
	int delta_w1_row = pos2[0] - pos1[0];
	int delta_w2_row = pos0[0] - pos2[0];

	int bias0 = is_flat_top_or_left_edge(pos0, pos1) ? 0 : -1; 
	int bias1 = is_flat_top_or_left_edge(pos1, pos2) ? 0 : -1; 
	int bias2 = is_flat_top_or_left_edge(pos2, pos0) ? 0 : -1; 

	int xmin = std::min(std::min(pos0[0], pos1[0]), pos2[0]);
	int ymin = std::min(std::min(pos0[1], pos1[1]), pos2[1]);
	int xmax = std::max(std::max(pos0[0], pos1[0]), pos2[0]);
	int ymax = std::max(std::max(pos0[1], pos1[1]), pos2[1]);

	int area = edge_cross(pos0, pos1, pos2);

	iVec2 p0 = { xmin, ymin};
	int w0_row = edge_cross(pos0, pos1, p0) + bias0;
	int w1_row = edge_cross(pos1, pos2, p0) + bias1;
	int w2_row = edge_cross(pos2, pos0, p0) + bias2;

	for (int y = ymin; y <= ymax; y++) {
		int w0 = w0_row;
		int w1 = w1_row;
		int w2 = w2_row;
		for (int x = xmin; x <= xmax; x++) {
			std::size_t pixelIdx = y * cols + x;
			
			// Barycentric coordinates in the triangle
			float alpha = std::abs(static_cast<float>(w1) / area);
			float beta = std::abs(static_cast<float>(w2) / area);
			float gamma = std::abs(static_cast<float>(w0) / area);
		
			// Is inside triangle?
			if (w0 <= 0 && w1 <= 0 && w2 <= 0) {
				Vec3 color {
					(alpha * v0.color[0]) + (beta * v1.color[0]) + (gamma * v2.color[0]),
					(alpha * v0.color[1]) + (beta * v1.color[1]) + (gamma * v2.color[1]),
					(alpha * v0.color[2]) + (beta * v1.color[2]) + (gamma * v2.color[2])
				};
				iVec2 texelPos {
					(alpha * v0.texPos[0] + beta * v1.texPos[0] + gamma * v2.texPos[0]) * texture.width,
					(alpha * v0.texPos[1] + beta * v1.texPos[1] + gamma * v2.texPos[1]) * texture.height
				};
				iVec3 texelColor = texture.GetPixel(texelPos);	
				Vec3 finalColor {
					color[0] * (texelColor[0] / 255.0f),
					color[1] * (texelColor[1] / 255.0f),
					color[2] * (texelColor[2] / 255.0f)
				};
				std::string colorCode = getColorEscapeCode(finalColor);
				memcpy(pixels[pixelIdx].colorCode, colorCode.c_str(), colorCode.size());
				pixels[pixelIdx].pixel = '+';
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
	std::cout << SWITCH_TO_ALT_TERMINAL << WIPE_TERMINAL;

	// get terminal dimensions
	struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
	std::size_t pixelCount = w.ws_col * w.ws_row;
	
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
		rasterize_triangle(texture, pixels, vertices[0], vertices[1], vertices[2], w.ws_row, w.ws_col);
		rasterize_triangle(texture, pixels, vertices[3], vertices[2], vertices[1], w.ws_row, w.ws_col);	

		// print
		std::vector<char> temp(sizeof(Pixel) * pixels.size() + 1);
		memcpy(temp.data(), pixels.data(), sizeof(Pixel) * pixels.size());
		temp[sizeof(Pixel) * pixels.size()] = '\0';
		std::cout << temp.data();
    }

	// restore the previous terminal
	std::cout << RESTORE_TERMINAL;	
	return 0;
}
