macro_rules! ImplCopy {
    ($(#[$meta:meta])* $vis:vis struct $name:ident {
        $($(#[$vmeta:meta])* $vvis:vis $vname:ident: $vtype:ty),*
        $(,)?
    }) => {
        $(#[$meta])*
        $vis struct $name {
            $($(#[$vmeta])* $vvis $vname: $vtype,)*
        }
        impl Clone for $name {
            fn clone(&self) -> Self {
                Self {
                    $($vname: self.$vname,)*
                }
            }
        }
        impl Copy for $name {
        }
    };
    ($(#[$meta:meta])* $vis:vis enum $name:ident {
        $($(#[$vmeta:meta])* $vname:ident($vtype:ty),)*
    }) => {
        $(#[$meta])*
        $vis enum $name {
            $($(#[$vmeta])* $vname($vtype),)*
        }
        impl Clone for $name {
            fn clone(&self) -> Self {
                match &self {
                    $(Self::$vname(t) => Self::$vname(t.clone()),)*
                }
            }
        }
        impl Copy for $name {
        }
    };
}

pub(crate) use ImplCopy;

macro_rules! IntEnum {
    ($(#[$meta:meta])* $vis:vis enum $name:ident {
        $($(#[$vmeta:meta])* $vname:ident $(= $val:expr)?),*
        $(,)?
    }) => {
        $(#[$meta])*
        $vis enum $name {
            $($(#[$vmeta])* $vname $(= $val)?,)*
        }
        impl Clone for $name {
            fn clone(&self) -> Self {
                *self
            }
        }
        impl Copy for $name {
        }
        impl std::convert::TryFrom<usize> for $name {
            type Error = ();
            fn try_from(value: usize) -> Result<Self, Self::Error> {
                const TABLE: [$name; 0$(+($name::$vname as usize == $name::$vname as usize) as usize)*] = [
                    $($name::$vname,)*
                ];
                match TABLE.get(value) {
                    Some(t) => Ok(*t),
                    None => Err(()),
                }
            }
        }
        impl std::convert::TryFrom<u32> for $name {
            type Error = ();
            fn try_from(value: u32) -> Result<Self, Self::Error> {
                Self::try_from(value as usize)
            }
        }
    };
}

pub(crate) use IntEnum;