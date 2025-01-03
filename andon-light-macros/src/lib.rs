use proc_macro::TokenStream;
use syn::DeriveInput;

// Highly inspired by strum_macros
#[proc_macro_derive(ErrorCodes, attributes(code))]
pub fn error_codes_derive(input: TokenStream) -> TokenStream {
    let ast: DeriveInput = syn::parse(input).unwrap();

    let ident = &ast.ident;

    let variants = match ast.data {
        syn::Data::Enum(enum_item) => enum_item.variants,
        _ => panic!("ErrorCodes works only on Enums"),
    };

    let len = variants.len();

    let mut to_str_arms = Vec::new();
    let mut from_str_arms = Vec::new();
    let mut description_arms = Vec::new();

    for variant in variants {
        let variant_ident = &variant.ident;
        match &variant.fields {
            syn::Fields::Unit => {}
            _ => panic!("ErrorCodes only works on unit variants"),
        }

        let variant_stringified = variant_ident.to_string();
        let variant_meta = variant
            .attrs
            .iter()
            .find(|attr| attr.path().is_ident("code"));

        let mut message: Option<String> = None;
        match variant_meta {
            Some(attr) => {
                attr.parse_nested_meta(|meta| {
                    if meta.path.is_ident("message") {
                        let value = meta.value()?; // this parses the `=`
                        let s: syn::LitStr = value.parse()?;
                        message = Some(s.value());
                        Ok(())
                    } else {
                        Err(meta.error("unsupported attribute"))
                    }
                })
                .unwrap();
            }
            None => panic!(
                "All variants of ErrorCodes must have a message attribute (#[code(message = \"...\")])"
            )
        };

        let message = match message {
            Some(value  ) => {
                if value.is_empty() {
                    panic!("ErrorCodes message attribute must not be empty")
                }
                value
            }
            None => panic!(
                "All variants of ErrorCodes must have a message attribute (#[code(message = \"...\")])"
            ),
        };

        to_str_arms.push(quote::quote! {
            #ident::#variant_ident => #variant_stringified,
        });
        from_str_arms.push(quote::quote! {
            stringify!(#variant_ident) => Ok(#ident::#variant_ident),
        });
        description_arms.push(quote::quote! {
            #ident::#variant_ident => #message,
        });
    }

    quote::quote! {
        impl #ident {
            const SIZE: usize = #len;

            pub fn as_str(&self) -> &'static str {
                match self {
                    #(#to_str_arms)*
                }
            }

            pub fn from_str(value: &str) -> Result<Self, &'static str> {
                match value {
                    #(#from_str_arms)*
                    _ => Err("Unknown error code"),
                }
            }

            pub fn description(&self) -> &'static str {
                match self {
                    #(#description_arms)*
                }
            }
        }
    }
    .into()
}
