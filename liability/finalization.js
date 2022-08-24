import Web3 from 'web3';
import { readFileSync } from "fs";
//import { off } from 'process';

const web3 = new Web3("https://proxy.devnet.neonlabs.org/solana")

let args = process.argv.slice(2)
let config = readFileSync(`${args[0]}/config/config`)
let json_config = JSON.parse(config)

let result_ipfs = args[1]
let liability_address = args[2]
const lighthouse_address = json_config["lighthouse_address"]

const main_account = json_config["main_account"]
const main_account_pk = json_config["main_account_pk"]
const validator_account_pk = json_config["validator_account_pk"]

async function liabilityFinalization(web3) {

    // let result = web3.utils.randomHex(34);
    let abi = readFileSync(`${args[0]}/liability/abi/Lighthouse.json`)
    let json_abi = JSON.parse(abi)
    let lighthouse = await new web3.eth.Contract(json_abi, lighthouse_address)
    let result =
    {
        address: liability_address
        , result: result_ipfs
        , success: true
    }
    const hash = web3.utils.soliditySha3(
        { t: 'address', v: result.address },
        { t: 'bytes', v: web3.utils.toHex(result.result) },
        { t: 'bool', v: result.success }
    );
    // console.log(`hash finalise: ${hash}`)
    console.log(result)
    const signature = await web3.eth.accounts.sign(hash, validator_account_pk);
    // console.log(`signature ${signature.signature}`)
    let tx = await lighthouse.methods.finalizeLiability(result.address, web3.utils.toHex(result.result), result.success, signature.signature).send({ from: main_account, gas: 1000000000 })
    console.log(tx.transactionHash)
}

web3.eth.accounts.wallet.add(validator_account_pk)
web3.eth.accounts.wallet.add(main_account_pk)
web3.eth.defaultAccount = main_account
await liabilityFinalization(web3)