//! `Rng`: deterministic pseudo-random numbers for the simulators, so a simulated run's junk
//! readings, camera intervals and losses are the same every time it runs (xorshift64*).

pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in [0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// True with probability `p`.
    pub fn chance(&mut self, p: f64) -> bool {
        self.unit() < p
    }

    /// Roughly normal (a sum of uniforms): mean 0, standard deviation `sd`.
    pub fn jitter(&mut self, sd: f64) -> f64 {
        let sum: f64 = (0..12).map(|_| self.unit()).sum();
        (sum - 6.0) * sd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_same_seed_gives_the_same_numbers() {
        let (mut a, mut b) = (Rng::new(7), Rng::new(7));
        assert!((0..100).all(|_| a.next_u64() == b.next_u64()));
    }

    #[test]
    fn chance_comes_out_near_its_probability() {
        let mut r = Rng::new(1);
        let hits = (0..100_000).filter(|_| r.chance(0.3)).count();
        assert!((29_000..31_000).contains(&hits), "{hits}");
    }
}
