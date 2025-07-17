use embedded_graphics_core::pixelcolor::Rgb888;
use embedded_graphics_core::prelude::*;
use smart_leds::{brightness, SmartLedsWrite, RGB8};

pub struct PovDisplay<L: SmartLedsWrite, const WIDTH: usize, const HEIGHT: usize> {
    leds: L,
    pub current_col: usize,
    buffer: [[Rgb888; HEIGHT]; WIDTH],
}

impl<L: SmartLedsWrite, const WIDTH: usize, const HEIGHT: usize> PovDisplay<L, WIDTH, HEIGHT> {
    pub fn new(leds: L) -> Self {
        Self {
            leds,
            current_col: 0,
            buffer: [[Rgb888::default(); HEIGHT]; WIDTH],
        }
    }

    pub fn reset_col(&mut self) {
        self.current_col = 0;
    }

    pub fn next_col(&mut self) {
        self.current_col = (self.current_col + 1) % WIDTH;
    }

    pub fn previous_col(&mut self) {
        self.current_col = (self.current_col - 1) % WIDTH;
    }

    pub fn show_colour(&mut self, colour: RGB8, num_leds: usize, bright: u8)
    where
        L::Color: From<RGB8>,
        L::Error: core::fmt::Debug,
    {
        let mut colours = [smart_leds::colors::BLACK; HEIGHT];

        for i in 0..num_leds {
            colours[i] = colour;
        }

        self.leds
            .write(brightness(colours.into_iter(), bright))
            .expect("Failed to write to leds");
    }

    pub fn clear(&mut self) {
        self.buffer = [[Rgb888::default(); HEIGHT]; WIDTH];
    }

    pub fn flush(&mut self, bright: u8)
    where
        L::Color: From<RGB8>,
        L::Error: core::fmt::Debug,
    {
        let colours = self.buffer[self.current_col].map(|x| RGB8::new(x.r(), x.g(), x.b()));

        self.leds
            .write(brightness(colours.into_iter(), bright))
            .expect("Failed to write to leds");
    }
}

#[derive(Debug)]
#[allow(dead_code)]
pub enum PovDisplayError {
    PixelOutOfBounds(i32, i32),
}

impl<L: SmartLedsWrite, const WIDTH: usize, const HEIGHT: usize> DrawTarget
    for PovDisplay<L, WIDTH, HEIGHT>
{
    type Color = Rgb888;
    type Error = PovDisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, colour) in pixels {
            if (0..(WIDTH as i32)).contains(&coord.x) && (0..(HEIGHT as i32)).contains(&coord.y) {
                self.buffer[coord.x as usize][coord.y as usize] = colour;
            } else {
                return Err(PovDisplayError::PixelOutOfBounds(coord.x, coord.y));
            }
        }

        Ok(())
    }
}

impl<L: SmartLedsWrite, const WIDTH: usize, const HEIGHT: usize> OriginDimensions
    for PovDisplay<L, WIDTH, HEIGHT>
{
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}
