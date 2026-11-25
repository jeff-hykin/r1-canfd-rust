//! Big-endian field reads that return `None` past the end of a payload instead of panicking.

pub(crate) fn u16_be(data: &[u8], offset: usize) -> Option<u16> {
    Some(u16::from_be_bytes(data.get(offset..offset + 2)?.try_into().ok()?))
}

pub(crate) fn i16_be(data: &[u8], offset: usize) -> Option<i16> {
    Some(i16::from_be_bytes(data.get(offset..offset + 2)?.try_into().ok()?))
}

pub(crate) fn u32_be(data: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_be_bytes(data.get(offset..offset + 4)?.try_into().ok()?))
}

pub(crate) fn u32_le(data: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_le_bytes(data.get(offset..offset + 4)?.try_into().ok()?))
}

/// `N` consecutive big-endian i16s starting at `offset`.
pub(crate) fn i16s_be<const N: usize>(data: &[u8], offset: usize) -> Option<[i16; N]> {
    let mut words = [0i16; N];
    for (index, word) in words.iter_mut().enumerate() {
        *word = i16_be(data, offset + 2 * index)?;
    }
    Some(words)
}

/// Bytes from `offset` to the end (empty when the payload is shorter).
pub(crate) fn tail(data: &[u8], offset: usize) -> Vec<u8> {
    data.get(offset..).unwrap_or_default().to_vec()
}

/// A fixed-size array copied out of `data` at `offset`.
pub(crate) fn array<const N: usize>(data: &[u8], offset: usize) -> Option<[u8; N]> {
    data.get(offset..offset + N)?.try_into().ok()
}
