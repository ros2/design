---
layout: default
title: ROS2 DDS security PKCS#11 URI support
abstract:
  The [DDS-Security specification][dds_security] defines the use of Hardware Security Modules (HSM) and PKCS#11 URIs as an alternative to private keys and certificates stored in the file system. Current implementation only supports these tokens to be directly stored in the file system as `.pem` files. This is a design proposal to support PKCS#11 URIs.
author: '[Iker Luengo](https://github.com/IkerLuengo)'
published: false
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Scope

The [DDS-Security specification][dds_security] requires the security documents (private keys, certificates, governance and permissions) to be provided to the DDS implementation within the participant's `properties`. It defines a set of reserved properties, that must hold the URI of the corresponding document. However, it allows different URI schemes to be used. Specifically, in the case of certificates and private keys, it defines the support for:

- file scheme (`file:/keystore/enclaves/foo/key.pem`).
- PKCS#11 scheme (`pkcs11:object=MyParticipantPrivateKey;type=private`).
- data scheme (`data:,-----BEGIN RSA PRIVATE KEY----- MIIEpAIBAAKCAQEA3HIh...AOBaaqSV37XBUJg== -----END RSA PRIVATE KEY-----`).

Current RCL impementation only support the `file` scheme. Furthermore, it searches these security files in an enclave subdirectory within the reserved `enclaves` subfolder of the root keystore, corresponding to the fully-qualified path of every enclave.
For example, for the `/foo/bar` enclave, the directory structure would look like:

    <root>
    ├── enclaves
    │   └── foo
    │       └── bar
    │           ├── cert.pem
    │           ├── key.pem
    │           ├── ...
    └── public
        ├── ...


Note that it also requires the names of the files to be the ones expected. Specifically, all certificate and key files must have the `.pem` extension. In order to configure the security properties of a DDS participant, the path of the appropriate file in the enclave directory is added as the `file` URI of the corresponding property. For example, for the private key in the authentication plugin:

    dds.sec.auth.private_key = file:<root>/enclaves/foo/bar/key.pem

## Proposal

### Goals

Support PKCS#11 URIs for certificates and key files. `data` URIs are out of scope of this proposal.

### Specification

We want to keep the current keystore structure as much as possible, as this will enable to keep all the current implementation regarding the enclave management and the CLI features that help setting up the keystore. No changes should be needed to systems that do not use the PKCS#11 scheme.

The problem then is how to let the RMW implementation when we want to use a `file` URI and when a `pkcs11` URI; and how to provide the values of these URIs (i.e., the file path in the case of `file` and the token name in the case of `pkcs11`).

This can be solved if we allow for the key and certificate files in the enclave to have `.pem` or `.p11` extensions. Files with `.p11` will contain the PKCS#11 URI instead of the actual document data. Then, the RMW can be aware of the file extension and set the security property accordingly. Taking the private key in the authentication plugin as an example:

1. The RMW will look for a file with name `key.pem` or `key.p11` in the enclave.
1. If there is a `key.pem` file, it keeps the current behavior, and sets the value of the property to the path of the file.
1. Otherwise, if there is a `key.p11` file, it must load the content of the file, and set this content as the value of the property. Some check can be done at this point, e.g., assert that the file contains a PKCS#11 URI (i.e., starts with `pkcs11:`).

With this proposal, current RMW implementations do not need to be updated if no support for PKCS#11 URIs is planned. Existing use SROS2 projects that do not use PKCS#11 URIs will continue to work with both the legacy or the updated implementation, New projects that want to use PKCS#11 URIs will fail unless the RMW supports `.p11` extension files as described in this proposal.



[dds_security]: https://www.omg.org/spec/DDS-SECURITY/1.1/PDF
